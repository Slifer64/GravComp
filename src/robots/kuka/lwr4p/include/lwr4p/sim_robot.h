#ifndef LWR4P_SIM_ROBOT_H
#define LWR4P_SIM_ROBOT_H

#include <lwr4p/robot_arm.h>

namespace as64_
{

namespace lwr4p_
{

class SimRobot : public RobotArm
{
public:
  SimRobot();
  SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);
  SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link, double ctrl_cycle);

  bool isOk() const override;
  void enable() override;
  void update() override;
  void setMode(const lwr4p_::Mode &m) override;

  void setJointsPosition(const arma::vec &j_pos) override;
  void setJointsVelocity(const arma::vec &j_vel) override;
  void setTaskVelocity(const arma::vec &task_vel) override;
  void setJointsTorque(const arma::vec &j_torques) override;
  void setTaskPose(const arma::mat &task_pose) override;
  void setWrench(const arma::vec &wrench) override;

  arma::vec getJointsPosition() const override;
  arma::mat getTaskPose() const override;
  arma::vec getTaskPosition() const override;
  arma::mat getTaskRotm() const override;
  arma::vec getTaskQuat() const override;
  arma::mat getJacobian() const override;
  arma::mat getEEJacobian() const override;
  arma::vec getJointsTorque() const override;
  arma::vec getExternalWrench() const override;
  arma::vec getJointExternalTorque() const override;

  void setCartStiffness(const arma::vec &cart_stiff) override;
  void setCartDamping(const arma::vec &cart_damp) override;

  void initJointsPosition(const arma::vec j_pos0);

private:

  arma::vec cart_stiff;
  arma::vec cart_damp;

  void initSimRobot();
  void setJointsPositionHelper(const arma::vec &j_pos);

  arma::vec joint_pos;

  void stop() override;
  void protectiveStop() override;

  lwr4p_::Timer timer;
  unsigned long update_time;

  ros::Subscriber jState_sub; ///< joint state subscriber
  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);
};

}; // namespace lwr4p_

}; // namespace as64_

#endif // LWR4P_SIM_ROBOT_H
