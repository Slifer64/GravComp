#ifndef AS64_ROBOT_WRAPPER_INV_KINEMATICS_INTERFACE_H
#define AS64_ROBOT_WRAPPER_INV_KINEMATICS_INTERFACE_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <thread>
#include <armadillo>

#include <robo_lib/singular_value_filter.h>

using namespace as64_;

namespace rw_
{

class InvKinematicsInterface
{
public:
  InvKinematicsInterface();
  ~InvKinematicsInterface();

  void initInvKinematicsInterface();

  void setJacobInvFilter(double sigma_min = 0.1, double shape_f = 19.9);

  virtual arma::mat getJacobian() const = 0;

  arma::mat getInvJacobian() const;

protected:

  std::shared_ptr<robo_::SingularValueFilter> svf;
  bool use_svf;

};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_INV_KINEMATICS_INTERFACE_H
