#ifndef UR_ROBOT_H
#define UR_ROBOT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <memory>

#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/WrenchStamped.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <sensor_msgs/JointState.h>
//#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/ur_driver.h>

#include <armadillo>

#include <ur_robot/robot_arm.h>


#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

namespace ur_
{

#define UR_Robot_fun_ std::string("[ur_::Robot::") + __func__ + "]: "

class Robot : public RobotArm
{
public:
  Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
       const std::string &host_ip, const std::string &robot_ip, int reverse_port);
  Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
        const std::string &host_ip, const std::string &robot_ip, int reverse_port);
  ~Robot();

  bool isOk() const override;
  void enable() override;
  void update() override;
  void setFreedriveMode() override;
  void setNormalMode() override;

  //                frame           selection        forces     type         limits
  //force_mode(p[0.1,0,0,0,0.785], [1,0,0,0,0,0], [20,0,40,0,0,0], 2, [.2,.1,.1,.785,.785,1.57])

  void force_mode(const arma::vec &select, double damping, const arma::vec &frame, const arma::vec &wrench, const::arma::vec &limits);

  arma::vec getJointsPosition() const override
  { return ur_driver->getJointPos(); }

  arma::vec getJointsVelocity() const override
  { return ur_driver->getJointVel(); }

  arma::vec getTaskVelocity() const override
  { return ur_driver->getTcpVel(); }

  arma::mat getEEJacobian() const override
  {
    // std::unique_lock<std::mutex> lck(robot_state_mtx);
    arma::mat R = getTaskPose().submat(0,0,2,2);
    arma::mat Jrobot = getJacobian();
    arma::mat Jee(6, N_JOINTS);
    Jee.submat(0,0,2,N_JOINTS-1) = R * Jrobot.submat(0,0,2,N_JOINTS-1);
    Jee.submat(3,0,5,N_JOINTS-1) = R * Jrobot.submat(3,0,5,N_JOINTS-1);

    return Jee;
  }

  arma::mat getJacobian() const override
  { return robot_urdf->getJacobian(getJointsPosition()); }

  arma::mat getTaskPose() const override
  {
    arma::mat output = arma::mat().eye(4, 4);
    output.submat(0,0,2,2) = getTaskRotm();
    output.submat(0,3,2,3) = getTaskPosition();
    return output;
  }

  arma::vec getTaskPosition() const override
  {
    //return ur_driver->getTcpPos();
    return robot_urdf->getTaskPosition(getJointsPosition());
  }

  arma::mat getTaskRotm() const override
  {
    return quat2rotm(getTaskQuat());
  }

  arma::vec getTaskQuat() const override
  {
    //return ur_driver->getTcpQuat();
    return robot_urdf->getTaskQuat(getJointsPosition());
  }

  /* Returns the wrench measured by the robot sensor, located at the TCP, expressed in the robot's base frame
   */
  arma::vec getTcpWrench() const override
  {
    return ur_driver->getTcpWrench();
  }

  /* Returns the wrench measured by the robot sensor, expressed in the robot's sensor frame, which is the TCP
   */
  arma::vec getSensorWrench() const
  {
    arma::mat R = quat2rotm( ur_driver->getTcpQuat() ).t();
    arma::vec F = getTcpWrench();
    return arma::join_vert( R*F.subvec(0,2), R*F.subvec(3,5) );
  }

  arma::vec getJointsTorque() const
  {
    return ur_driver->getJointTorque();
  }

  // double servo_a, servo_v, servo_T, servo_lookahead_time, servo_gain;

  void setJointsPosition(const arma::vec &j_pos) override
  {
    ur_driver->setJointsPosition(j_pos);
    // this->servoj(j_pos, servo_a, servo_v, servo_T, servo_lookahead_time, servo_gain);
  }

  void setJointsVelocity(const arma::vec &j_vel) override
  {
    ur_driver->setJointsVelocity(j_vel, 5);
    // this->speedj(j_vel, 5, servo_T);
  }

  void setTaskVelocity(const arma::vec &task_vel) override
  {
    ur_driver->setTaskVelocity(task_vel, 20);
    // this->speedl(task_vel, 20, servo_T);
  }

  void setTaskPose(const arma::vec &pose) override
  {
    // arma::vec pos = pose.subvec(0,2);
    // arma::vec quat = pose.subvec(3,6);
    //
    // // convert quaternion to axis*angle representation
    // arma::vec ax_ang = arma::vec().zeros(3);
    // arma::vec v = quat.subvec(1,3);
    // double u = quat(0);
    // double v_norm = arma::norm(v);
    // if (v_norm > 1e-16) ax_ang = 2*std::atan2(v_norm,u)*v/v_norm;
    //
    // ur_driver->setTaskPose(arma::join_vert(pos, ax_ang));

    arma::mat Pose = arma::mat().eye(4,4);
    Pose.submat(0,3,2,3) = pose.subvec(0,2);
    Pose.submat(0,0,2,2) = quat2rotm(pose.subvec(3,6));

    bool found_sol = false;
    arma::vec qd = robot_urdf->getJointsPosition(Pose, getJointsPosition(), &found_sol);

    if (!found_sol) std::cerr << UR_Robot_fun_ << "Failed to find inverse solution...\n";

    setJointsPosition(qd);
  }

  void biasFtSensor() { ur_driver->biasFtSensor(); /*ur_driver->setUrScriptCmd("zero_ftsensor()\n");*/ }

  void movej(const arma::vec &q, double a, double v, double t, double r=0);

private:

  /** Sends a urscript command to set the robot in control mode 'mode'. */
  void command_mode(const std::string &mode) const;

  void protectiveStop() override;

  void initRobot(const std::string &host_ip, const std::string &robot_ip, int reverse_port);

  arma::vec last_joint_pos;

  UrDriver *ur_driver;
  void runUrDriver();
  std::thread ur_driver_thr;
  std::string robot_ip_;
  std::string host_ip_;
  int reverse_port;
  Semaphore shutdown_sem;

  std::string ur_script; ///< string that can store an entire URscript file.

  /** Prints a vector as a URscript string. */
  std::string print_vector(const arma::vec &v) const
  {
    std::ostringstream out;
    out << "[" << v(0);
    for (int i=1;i<v.size();i++) out << "," << v(i);
    out << "]";

    return out.str();
  }

  void servoj(const arma::vec &q, double a, double v, double t, double lookahead_time=0.1, double gain=300);

  void movel(const arma::vec &p, double a, double v, double t, double r=0);
  void speedj(arma::vec dq, double a, double t);
  void speedl(arma::vec dp, double a, double t);
  void stopj(double a);
  void stopl(double a);
  void sleep(double t);
  void powerdown();

};

}  // namespace ur_

#endif  // UR_ROBOT_H
