#include <cell.h>


template <typename PointT> 
class NDTEDGridMap: public boost::noncopyable
{
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  // typedef typename NormalDistCell<PointT> Cell;

  public:
    NDTEDGridMap (const Eigen::Vector2f& origin,
                  const Eigen::Vector2f& step,
                  const Eigen::Vector2i& size,
                  const size_t min_n = 3)
        : origin_ (origin), step_ (step), size_ (size),
          cells_ (size_[0], size_[1]), 
          min_n_(min_n), added_(false)
    {
      Eigen::Vector2f size_f = size_.cast<float> ();
      for (size_t i = 0; i < size_[0]; i++) {
        for (size_t j = 0; j < size_[1]; j++) {
          Eigen::Vector3d mean (origin_[0] + (i - (size_f[0] / 2.0)) * step_[0], 
                                origin_[1] + (j - (size_f[1] / 2.0)) * step_[1], 
                                0);
          Eigen::Matrix3d cov;
          cov <<
            step_[0]/4,           0,                    0,
            0,                    step_[1]/4,           0,
            0,                    0,                    20/4;

          cells_.coeffRef (i, j).initCell (mean, cov);
          std::cout << i << ", " << j << std::endl;
        }
      }
    }

    void
    addCloud (PointCloudConstPtr cloud)
    {
      NormalDistCell<PointT>* n;
      for (size_t i = 0; i < cloud->size (); i++)
        if (n = getCell (cloud->at (i))) n->addPoint (cloud->at(i));
      added_ = true;
    }

    void
    estimateParam ()
    {
      if (!added_) return;
      for (size_t i = 0; i < size_[0]; i++)
        for (size_t j = 0; j < size_[1]; j++)
          cells_.coeffRef (i, j).estimateParams (5, 0.001);
      added_ = false;
    }

    PointCloud
    getMapCloud ()
    {
    PointCloud mean_cloud;
      for (size_t i = 0; i < size_[0]; i++) {
        for (size_t j = 0; j < size_[1]; j++) {
          PointT mean_point;
          Eigen::Vector3d mean = cells_.coeffRef (i, j).getMean();
          mean_point.x = mean[0];
          mean_point.y = mean[1];
          mean_point.z = mean[2];
          mean_cloud.push_back (mean_point);
        }
      }

      return mean_cloud;
    }
    
    ValueAndDerivatives<3,double>
    test (PointCloudConstPtr cloud, const double& cos_theta, const double& sin_theta) const
    {
      NormalDistCell<PointT>* n;
      ValueAndDerivatives<3,double> score = ValueAndDerivatives<3,double>::Zero ();
      for (size_t i = 0; i < cloud->size (); i++)
        if (n = getCell (cloud->at (i)))
          score += n->score (cloud->at (i), cos_theta, sin_theta);
      
      return score;
    }

  protected:
    NormalDistCell<PointT>* 
    getCell (PointT const& p) const
    {
      Eigen::Vector2f idxf(p.x, p.y);
      for (size_t i = 0; i < 2; i++)
        idxf[i] = floor((p.getVector3fMap ()[i] - origin_[i]) / step_[i] + 0.5) + size_[i] / 2;
      Eigen::Vector2i idxi = idxf.cast<int> ();
      if(!(checkCellID (idxi))) return NULL;
      return const_cast<NormalDistCell<PointT>*> (&cells_.coeffRef (idxi[0], idxi[1]));
    }

    bool
    checkCellID (Eigen::Vector2i id) const
    {
      for (size_t i = 0; i < 2; i++)
        if (id[i] >= size_[i] || id[i] < 0) return false;
      return true;
    }


    size_t min_n_;
    Eigen::Vector2f min_;
    Eigen::Vector2f max_;
    Eigen::Vector2f origin_;
    Eigen::Vector2f step_;
    Eigen::Vector2i size_;

    bool added_;

    Eigen::Matrix<NormalDistCell<PointT>, Eigen::Dynamic, Eigen::Dynamic> cells_;
};

namespace Eigen
{
  template<typename PointT> struct NumTraits<NormalDistCell<PointT> >
  {
    typedef double Real;
    typedef double Literal;
    static Real dummy_precision () { return 1.0; }
    enum {
      IsComplex = 0,
      IsInteger = 0,
      IsSigned = 0,
      RequireInitialization = 1,
      ReadCost = 1,
      AddCost = 1,
      MulCost = 1
    };
  };
}
