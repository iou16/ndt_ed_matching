#include <Eigen/Core>


template<unsigned N=3, typename T=double>
struct ValueAndDerivatives
{
  ValueAndDerivatives () : hessian (), grad (), value () {}

  Eigen::Matrix<T, N, N> hessian;
  Eigen::Matrix<T, N, 1>    grad;
  T value;
  
  static ValueAndDerivatives<N,T>
  Zero ()
  {
    ValueAndDerivatives<N,T> r;
    r.hessian = Eigen::Matrix<T, N, N>::Zero ();
    r.grad    = Eigen::Matrix<T, N, 1>::Zero ();
    r.value   = 0;
    return r;
  }

  ValueAndDerivatives<N,T>&
  operator+= (ValueAndDerivatives<N,T> const& r)
  {
    hessian += r.hessian;
    grad    += r.grad;
    value   += r.value;
    return *this;
  }
};
