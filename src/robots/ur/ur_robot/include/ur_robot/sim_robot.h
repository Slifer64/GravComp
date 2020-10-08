#ifndef UR_SIM_ROBOT_H
#define UR_SIM_ROBOT_H

#include <ur_robot/robot_arm.h>

namespace ur_
{

class SimRobot : public RobotArm
{
public:
  SimRobot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link);
  SimRobot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link);

  bool isOk() const override;
  void enable() override;
  void update() override;
  void setFreedriveMode() override;
  void setNormalMode() override;

  void setJointsPosition(const arma::vec &j_pos) override;
  void setJointsVelocity(const arma::vec &j_vel) override;
  void setTaskVelocity(const arma::vec &task_vel) override;
  void setTaskPose(const arma::vec &task_pose) override;

  arma::vec getJointsPosition() const override;
  arma::vec getJointsVelocity() const override;
  arma::mat getTaskPose() const override;
  arma::vec getTaskPosition() const override;
  arma::mat getTaskRotm() const override;
  arma::vec getTaskQuat() const override;
  arma::vec getTaskVelocity() const override;
  arma::mat getJacobian() const override;
  arma::mat getEEJacobian() const override;
  arma::vec getTcpWrench() const override;

  void initJointsPosition(const arma::vec j_pos0);

private:

  bool is_freedrive_on;

  void initSimRobot();
  void setJointsPositionHelper(const arma::vec &j_pos);

  arma::vec joint_pos;

  void protectiveStop();

  ur_::Timer timer;
  unsigned long update_time;

  ros::Subscriber jState_sub; ///< joint state subscriber
  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);
};

}; // namespace ur_

#endif // UR_SIM_ROBOT_H
