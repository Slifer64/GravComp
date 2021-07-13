#ifndef AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H
#define AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H

#include <robot_wrapper/robot.h>

#include <string>
#include <armadillo>

namespace rw_
{

class Robot; // forward declaration

class AdmittanceCtrl
{
public:
  AdmittanceCtrl(rw_::Robot *robot);

  void init();

  void update();

  void loadParams(const std::string &filename);

  arma::vec getVelocity() const;

  void stop();

private:

  rw_::Robot *robot_;

  // controller state
  arma::vec p, p_dot, p_ddot; // cart pos state
  arma::vec Q, omega, omega_dot; // cart orient state

  // controller params
  arma::vec Mp; // cart pos stiffness
  arma::vec Dp; // cart pos damping

  arma::vec Mo; // cart orient stiffness
  arma::vec Do; // cart orient damping

  double a_f; // filtering coeff for wrench
  arma::vec Fext_prev;

  arma::vec ret_vel; // the admittance cart velocity calculated after each update

};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_ADMITTANCE_CONTROLLER_H
