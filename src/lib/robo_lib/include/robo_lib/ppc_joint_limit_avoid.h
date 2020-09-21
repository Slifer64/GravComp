#ifndef ROBO_LIB_PPC_JOINT_LIMIT_AVOIDANCE_H
#define ROBO_LIB_PPC_JOINT_LIMIT_AVOIDANCE_H

#include <iostream>
#include <armadillo>

namespace as64_
{

namespace robo_
{

class PPCJointLimAvoid
{
public:
  /**
  * @brief PPCJointLimAvoid::PPCJointLimAvoid
  */
  PPCJointLimAvoid();

  /**
  * @brief PPCJointLimAvoid::PPCJointLimAvoid
  * @param qmin, low limit of joints in a vector form
  * @param qmax, high limit of joints in a vector form
  * @param gain, gain for the attraction to the mean value
  */
  PPCJointLimAvoid(const arma::vec &qmin, const arma::vec &qmax);

  /**
  * @brief PPCJointLimAvoid::setGain, set one joint gain value
  * @param joint_index, index of the joint
  * @param gain, the gain value
  */
  void setGain(int joint_index, double gain);

  /**
  * @brief PPCJointLimAvoid::setGains, set all the joint gain values with the same value
  * @param gain, the gain value
  */
  void setGains(double gain);

  /**
  * @brief PPCJointLimAvoid::setGains, set all the joint gain values
  * @param gains, the gain values in vector form
  */
  void setGains(const arma::vec &gains);

  /**
  * @brief PPCJointLimAvoid::getControlSignal
  * @return the control signal, which is a velocity vector of dimension of N on the joint space
  * @param q_meas, measure joint position
  */
  arma::vec getControlSignal(const arma::vec &q_meas);


  /**
  * @brief PPCJointLimAvoid::init
  * @param qmin, low limit of joints in a vector form
  * @param qmax, high limit of joints in a vector form
  */
  void init(const arma::vec &qmin, const arma::vec &qmax);

  /**
  * @brief PPCJointLimAvoid::setLowLimit, set one joint limit value
  * @param joint_index, index of the joint
  * @param low_limit, the low limit value
  */
  void setLowLimit(int joint_index, double low_limit);

  /**
  * @brief PPCJointLimAvoid::setLowLimits, set all the joint limit values
  * @param qmins, joint low limits
  */
  void setLowLimits(const arma::vec &qmins);

  /**
  * @brief PPCJointLimAvoid::setHighLimit, set one joint limit value
  * @param joint_index, index of the joint
  * @param low_limit, the high limit value
  */
  void setHighLimit(int joint_index, double high_limit);

  /**
  * @brief PPCJointLimAvoid::setHighLimits, set all the joint limit values
  * @param qmaxs, joint high limits
  */
  void setHighLimits( const arma::vec &qmaxs);

private:

  void initAllVarsToZero();

  // Initialization for YuMi
  int Njoints_;

  // parameters
  arma::vec qmin_;
  arma::vec qmax_;
  arma::vec qmean_;
  arma::vec rho_;
  arma::vec kq_;
  double gain_;

  // variables
  arma::vec e_, c_, epsilon_, dT_;
  arma::vec sig_;

};

}; // namespace robo_

}; // namespace as64_

#endif // ROBO_LIB_PPC_JOINT_LIMIT_AVOIDANCE_H
