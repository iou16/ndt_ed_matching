#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <map.h>

class NDTEDMatching
{
  public:
    NDTEDMatching();
    ~NDTEDMatching();

    void startLiveMatching();
    void startReplay(const std::string bag_fname, std::string scan_topic);

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  private:
    bool sendTF(const ros::Time& t);
    bool getPose(tf::Pose& pose, const ros::Time& t, std::string frame_id);

	ros::NodeHandle private_nh_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber init_pose_sub_;

    ros::Publisher odom_pose_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher map_cloud_pub_;

    tf::TransformBroadcaster tf_b_;
    tf::TransformListener tf_l_;

    std::string base_frame_id_, odom_frame_id_, global_frame_id_;
    tf::Transform map_to_odom_;
    tf::Pose guess_pose_, prv_odom_pose_;

    NDTEDGridMap<pcl::PointXYZ> *map_;

    bool first_time_, set_init_pose_;

    Eigen::Vector3d newton_lambda_;
};

NDTEDMatching::NDTEDMatching() :
          map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))), 
          guess_pose_(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 )),
          prv_odom_pose_(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 )),
          first_time_(true), set_init_pose_(true), 
          newton_lambda_ (0.8, 0.8, 0.8)
{
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  std::string map_file;
  private_nh_.param("map_file", map_file, std::string("/home/humio/merge_cloud.pcd"));

  Eigen::Vector2f origin(0.0, 0.0);
  Eigen::Vector2f step(1.0, 1.0);
  Eigen::Vector2i size(200, 200);
  map_ = new NDTEDGridMap<pcl::PointXYZ>(origin, step, size);

  pcl::PointCloud<pcl::PointXYZ> map;
  pcl::PCDReader reader;
  if (map_file != "") {
    ROS_INFO_STREAM("Load map " << map_file);
    if (0 != reader.read<pcl::PointXYZ> (map_file, map)) {
      ROS_ERROR("Failed Loading");
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));
      map_->addCloud (map_ptr);
      map_->estimateParam ();
    }
  } else {
    ROS_ERROR("map file doesn't have name");
  }

  odom_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("odom_pose", 2, true);
  cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("test_cloud", 2, true);
  map_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 2, true);
}

NDTEDMatching::~NDTEDMatching()
{
  if (map_) delete map_;
};

void NDTEDMatching::startLiveMatching()
{
  cloud_sub_ = private_nh_.subscribe(std::string("/edgcloud"), 0, &NDTEDMatching::cloudCallback, this);
  init_pose_sub_ = private_nh_.subscribe(std::string("/initialpose"), 2, &NDTEDMatching::initPoseCallback, this);
}

void NDTEDMatching::startReplay(const std::string bag_fname, std::string scan_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scan_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  std::queue<sensor_msgs::PointCloud2::ConstPtr> s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_l_.setTransform(stampedTf);
      }
    }

    sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(s);
      }
    }

    while ((!s_queue.empty())&&(ros::ok()))
    {
      try
      {
        // tf::StampedTransform t;
        // tf_l_.lookupTransform(s_queue.front()->header.frame_id, odom_frame_id_, s_queue.front()->header.stamp, t);
        this->cloudCallback(s_queue.front());
        s_queue.pop();
      }
      catch(tf::ExtrapolationException& e)
      {
        break;
      }
    }
  }

  bag.close();
  ROS_INFO("ENDD");
}

void NDTEDMatching::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // ros::Time start = ros::Time::now();
  // static unsigned int step_num = 0;
  // static double add_time = 0.0;

  if(!(set_init_pose_)) return;

  if (first_time_){
    if(!sendTF(input->header.stamp)) return;

    if(!getPose(prv_odom_pose_, input->header.stamp, odom_frame_id_)) {
      ROS_WARN("Couldn't get robot's pose");
      return;
    }
    guess_pose_ = prv_odom_pose_;


    first_time_ = false;
    return;
  }

  sensor_msgs::PointCloud2 transformed_cloud;
  try {
    // tf_l_.waitForTransform(input->header.frame_id, 
    //                        base_frame_id_,
    //                        input->header.stamp,
    //                        ros::Duration(0.5));
    pcl_ros::transformPointCloud(base_frame_id_, *input, transformed_cloud, tf_l_);
  } catch(tf::TransformException e) {
    ROS_ERROR("Couldn't transforme point cloud");
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(transformed_cloud, *transformed_cloud_ptr);

  tf::Pose cr_odom_pose;
  if(!getPose(cr_odom_pose, input->header.stamp, odom_frame_id_)) {
    ROS_ERROR("Couldn't get robot's pose");
    return;
  }

  // tf::Pose delta_pose(tf::Quaternion(tf::createQuaternionFromYaw(angle_diff(tf::getYaw(cr_odom_pose.getRotation()), 
  //                                                                           tf::getYaw(prv_odom_pose_.getRotation())))),
  //                     tf::Vector3(cr_odom_pose.getOrigin().getX() - prv_odom_pose_.getOrigin().getX(),
  //                                 cr_odom_pose.getOrigin().getY() - prv_odom_pose_.getOrigin().getY(),
  //                                 0));

  tf::Pose delta_pose;
  tf::Transform odom_to_base(prv_odom_pose_.inverse().getRotation());
  delta_pose.setOrigin(odom_to_base * (cr_odom_pose.getOrigin() - prv_odom_pose_.getOrigin()));
  delta_pose.setRotation(cr_odom_pose.getRotation() * prv_odom_pose_.getRotation().inverse());

  tf::Transform base_to_global_ = tf::Transform(guess_pose_.getRotation());
  delta_pose.setOrigin(base_to_global_ * delta_pose.getOrigin());
  guess_pose_.setOrigin(guess_pose_.getOrigin() + delta_pose.getOrigin());
  guess_pose_.setRotation(guess_pose_.getRotation() * delta_pose.getRotation());

  geometry_msgs::PoseStamped p;
  tf::poseTFToMsg(guess_pose_, p.pose);
  p.header.frame_id = global_frame_id_;
  p.header.stamp = input->header.stamp;
  odom_pose_pub_.publish(p);

  prv_odom_pose_ = cr_odom_pose;

  Eigen::Vector3d transformation (
    guess_pose_.getOrigin().getX(),
    guess_pose_.getOrigin().getY(),
    tf::getYaw(guess_pose_.getRotation())
  );

  
  sensor_msgs::PointCloud2 tmp_cloud;
  pcl::toROSMsg (map_->getMapCloud (), tmp_cloud);
  tmp_cloud.header.frame_id = global_frame_id_;
  tmp_cloud.header.stamp = input->header.stamp;
  map_cloud_pub_.publish(tmp_cloud);


  int nr_iterations = 0;
  bool converged = false;
  Eigen::Vector3d previous_transformation;
  while ((!converged)&&(ros::ok()))
  // for (int step = 0; step < 35; step++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_ros::transformPointCloud(*transformed_cloud_ptr, *tmp_cloud_ptr, guess_pose_);
    pcl::toROSMsg(*tmp_cloud_ptr, tmp_cloud);
    tmp_cloud.header.frame_id = global_frame_id_;
    tmp_cloud.header.stamp = input->header.stamp;
    cloud_pub_.publish(tmp_cloud);
    const double cos_theta = std::cos (transformation[2]);
    const double sin_theta = std::sin (transformation[2]);
    previous_transformation = transformation;    

    ValueAndDerivatives<3, double> score = map_->test (tmp_cloud_ptr, cos_theta, sin_theta);
    
    if (score.value != 0)
    {
      Eigen::EigenSolver<Eigen::Matrix3d> solver;
      solver.compute (score.hessian, false);
      double min_eigenvalue = 0;
      for (int i = 0; i <3; i++)
        if (solver.eigenvalues ()[i].real () < min_eigenvalue)
            min_eigenvalue = solver.eigenvalues ()[i].real ();

      if (min_eigenvalue < 0)
      {
        double lambda = 1.1 * min_eigenvalue - 1;
        score.hessian += Eigen::Vector3d (-lambda, -lambda, -lambda).asDiagonal ();
        solver.compute (score.hessian, false);
      }
      assert (solver.eigenvalues ()[0].real () >= 0 &&
              solver.eigenvalues ()[1].real () >= 0 &&
              solver.eigenvalues ()[2].real () >= 0);
      
      Eigen::Vector3d delta_transformation (-score.hessian.inverse () * score.grad);
      transformation = transformation + newton_lambda_.cwiseProduct (delta_transformation);

      guess_pose_.setOrigin(tf::Vector3(transformation[0], transformation[1], 0));
      guess_pose_.setRotation(tf::Quaternion(tf::createQuaternionFromYaw(transformation[2])));
      tf::poseTFToMsg(guess_pose_, p.pose);
      p.header.frame_id = global_frame_id_;
      p.header.stamp = input->header.stamp;
      odom_pose_pub_.publish(p);
    }
    else
    {
      ROS_ERROR ("score.value if zero!");
      break;
    }
    nr_iterations++;

    Eigen::Vector3d transformation_delta = transformation - previous_transformation;
    double cos_angle = transformation[2];
    double translation_sqr = sqrt(transformation_delta[0] * transformation_delta[0] + transformation_delta[1] * transformation_delta[1]);
    if (nr_iterations >= 11 ||
        ((0.02 > 0 && translation_sqr <= 0.02) && (0.05 > 0 && cos_angle >= 0.05)) ||
        ((0.02 <= 0)                           && (0.05 > 0 && cos_angle >= 0.05)) ||
        ((0.02 > 0 && translation_sqr <= 0.02) && (0.05 <= 0)))
    {
      converged = true;
    }
  }
  // guess_pose_.setOrigin(tf::Vector3(transformation[0], transformation[1], 0));
  // guess_pose_.setRotation(tf::Quaternion(tf::createQuaternionFromYaw(transformation[2])));

  if(!sendTF(input->header.stamp)) return;

  // double time = ros::Time::now().toSec() - start.toSec();
  // // ROS_INFO_STREAM("time: " << time << " ");
  // add_time += time;
  // step_num++;
  // ROS_INFO_STREAM("ave_time: " << (add_time/(double)step_num));

  ROS_INFO("HOGE");
  return;
}

bool NDTEDMatching::sendTF(const ros::Time& t) {
  tf::Stamped<tf::Pose> odom_to_map;
  try
  {
    tf::Transform map_to_base = tf::Transform(tf::Quaternion(guess_pose_.getRotation()),
                                              tf::Point(guess_pose_.getOrigin()));
    tf::Stamped<tf::Pose> base_to_map_stamped (map_to_base.inverse(),
                                               t,
                                               base_frame_id_);
    tf_l_.transformPose(odom_frame_id_,
                        base_to_map_stamped,
                        odom_to_map);
  }
  catch(tf::TransformException)
  {
    ROS_WARN("Failed to subtract base to odom transform");
    return false;
  }

  map_to_odom_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));
  map_to_odom_ = map_to_odom_.inverse();
  tf::StampedTransform tmp_tf_stamped(map_to_odom_,
                                      t,
                                      global_frame_id_, odom_frame_id_);
  tf_b_.sendTransform(tmp_tf_stamped);

  return true;
}

bool NDTEDMatching::getPose(tf::Pose& pose, const ros::Time& t, std::string frame_id)
{
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), 
                                             tf::Vector3(0,0,0)), 
                               t, base_frame_id_);
  tf::Stamped<tf::Transform> tmp_pose;
  try
  {
    tf_l_.transformPose(frame_id, ident, tmp_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  pose = tf::Pose(tmp_pose.getRotation(), tmp_pose.getOrigin());

  return true;
}

void NDTEDMatching::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  ROS_INFO("set init pose");
  if(input->header.frame_id == "")
  {
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  else if(tf_l_.resolve(input->header.frame_id) != tf_l_.resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             input->header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  tf::poseMsgToTF(input->pose.pose, guess_pose_);

  set_init_pose_ = true;
  ROS_INFO("set init pose");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching");

  NDTEDMatching ndtedmacher;
  ndtedmacher.startLiveMatching();
  // ndtedmacher.startReplay("/home/humio/gaisyuu.bag", "/edgcloud");
  // ndtedmacher.startReplay("/home/humio/test.bag", "/edgcloud");
  ros::spin();

  return 0;
}
