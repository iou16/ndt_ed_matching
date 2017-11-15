#include <Eigen/Core>


#include <newton_values.h>


template <typename PointT>
class NormalDistCell
{
  public:
    NormalDistCell ()
      :n_ (0), mean_ (), covar_inv_ (), sx_ (), sxx_ ()
    {
    }

    void
    initCell (Eigen::Vector3d mean, Eigen::Matrix3d covar, double min_covar_eigvalue_mult = 0.001) {
      mean_ = mean;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (covar);
      if (solver.eigenvalues ()[0] < min_covar_eigvalue_mult * solver.eigenvalues ()[2])
      {
        Eigen::Matrix3d l = solver.eigenvalues ().asDiagonal ();
        Eigen::Matrix3d q = solver.eigenvectors ();
        l (0,0) = l (2,2) * min_covar_eigvalue_mult;
        if (solver.eigenvalues ()[1] < min_covar_eigvalue_mult * solver.eigenvalues ()[2]) l (1,1) = l (2,2) * min_covar_eigvalue_mult;
        covar = q * l * q.transpose ();
      }
      covar_inv_ = covar.inverse ();
      sx_  = Eigen::Vector3d::Zero ();
      sxx_ = Eigen::Matrix3d::Zero ();
      n_ = 0;
    }
        
    void
    addPoint (const PointT& point)
    {
      Eigen::Vector3d p (point.x, point.y, point.z);
      sx_  += p;
      sxx_ += p * p.transpose ();
      n_++;
    }
    
    void
    estimateParams (const size_t& min_n, double min_covar_eigvalue_mult = 0.001)
    {
      if (n_ >= min_n)
      {
        mean_ = sx_ / static_cast<double> (n_);
        // Eigen::Matrix3d covar = (sxx_ / static_cast<double> (n_)) - (mean_ * mean_.transpose ());

        // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (covar);
        // if (solver.eigenvalues ()[0] < min_covar_eigvalue_mult * solver.eigenvalues ()[2])
        // {
        //   Eigen::Matrix3d l = solver.eigenvalues ().asDiagonal ();
        //   Eigen::Matrix3d q = solver.eigenvectors ();
        //   l (0,0) = l (2,2) * min_covar_eigvalue_mult;
        //   if (solver.eigenvalues ()[1] < min_covar_eigvalue_mult * solver.eigenvalues ()[2]) l (1,1) = l (2,2) * min_covar_eigvalue_mult;
        //   covar = q * l * q.transpose ();
        // }
        // covar_inv_ = covar.inverse ();
      }
    }

    Eigen::Vector3d
    getMean ()
    {
      return mean_;
    }

    void
    setMean (Eigen::Vector3d mean)
    {
      mean_ = mean;
    }

    void addMean(Eigen::Vector3d mean)
    {
      mean_ += mean;
    }

    ValueAndDerivatives<3,double>
    score (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const
    {
      ValueAndDerivatives<3,double> r;
      const double x = transformed_pt.x;
      const double y = transformed_pt.y;
      const Eigen::Vector3d p (transformed_pt.x, transformed_pt.y, transformed_pt.z);
      const Eigen::Vector3d q = p - mean_;
      const Eigen::RowVector3d qt_cvi (q.transpose () * covar_inv_);
      const double exp_qt_cvi_q = std::exp (-0.5 * double (qt_cvi * q));
      r.value = -exp_qt_cvi_q;

      Eigen::Matrix3d jacobian;
      jacobian <<
        1, 0, -(x * sin_theta + y*cos_theta),
        0, 1,   x * cos_theta - y*sin_theta,
        0, 0, 0;
      
      for (size_t i = 0; i < 3; i++)
        r.grad[i] = double (qt_cvi * jacobian.col (i)) * exp_qt_cvi_q;
      
      const Eigen::Vector3d d2q_didj (
          y * sin_theta - x*cos_theta,
        -(x * sin_theta + y*cos_theta),
          0
      );

      for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++)
          r.hessian (i,j) = -exp_qt_cvi_q * (
            double (-qt_cvi*jacobian.col (i)) * double (-qt_cvi*jacobian.col (j)) +
            (-qt_cvi * ((i==2 && j==2)? d2q_didj : Eigen::Vector3d::Zero ())) +
            (-jacobian.col (j).transpose () * covar_inv_ * jacobian.col (i))
          );
      
      return r;
    }

protected:
    size_t n_;
    Eigen::Vector3d sx_;
    Eigen::Matrix3d sxx_;
    Eigen::Vector3d mean_;
    Eigen::Matrix3d covar_inv_;
    double d1_, d2_;
};
