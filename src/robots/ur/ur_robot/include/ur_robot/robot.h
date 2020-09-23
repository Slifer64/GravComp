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

namespace as64_
{

namespace ur_
{

#define UR_Robot_fun_ std::string("[ur_::Robot::") + __func__ + "]: "

class Robot : public RobotArm
{
public:
  Robot(urdf::Model &urdf_model, const std::string &base_link, const std::string &tool_link,
       const std::string &robot_ip, int reverse_port);
  Robot(const std::string &robot_desc_param, const std::string &base_link, const std::string &tool_link,
        const std::string &robot_ip, int reverse_port);
  ~Robot();

  bool isOk() const override;
  void enable() override;
  void update() override;
  void setFreedriveMode() override;
  void setNormalMode() override;

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
    return ur_driver->getTcpPos();
//    return robot_urdf->getTaskPosition(getJointsPosition());
  }

  arma::mat getTaskRotm() const override
  {
    return quat2rotm(getTaskQuat());
  }

  arma::vec getTaskQuat() const override
  {
    return ur_driver->getTcpQuat();
//    return robot_urdf->getTaskQuat(getJointsPosition());
  }

  arma::vec getTcpWrench() const override
  {
    arma::vec wrench = ur_driver->getTcpWrench();
    // std::cerr << "-------------------------------------\n";
    // std::cerr << "world force = " << wrench.subvec(0,2).t() << "\n";
    // arma::mat R = getTaskRotm();
    // wrench.subvec(0,2) = R.t()*wrench.subvec(0,2);
    // std::cerr << "local force = " << wrench.subvec(0,2).t() << "\n";
    // wrench.subvec(3,5) = R*wrench.subvec(3,5);
    // return ur_driver->getTcpWrench();
  }


  double servo_a, servo_v, servo_T, servo_lookahead_time, servo_gain;

  void setJointsPosition(const arma::vec &j_pos) override
  {
    // this->movej(j_pos, 30, 10, 0.03);
    this->servoj(j_pos, servo_a, servo_v, servo_T, servo_lookahead_time, servo_gain);
  }

  void setJointsVelocity(const arma::vec &j_vel) override
  {
//    this->speedj(j_vel, arma::max(arma::abs((j_vel-getJointsVelocity()))/ctrl_cycle), 0.03);
     this->speedj(j_vel, 5, 0.04);
  }

  void setTaskVelocity(const arma::vec &task_vel) override
  {
//    this->speedl(task_vel, arma::max(arma::abs((task_vel-getTaskVelocity()))/ctrl_cycle), 0.03);
    this->speedl(task_vel, 20, 0.04);
  }

  void setTaskPose(const arma::mat &input) override
  {
    throw std::runtime_error(UR_Robot_fun_ + "Not implemented...");
    // const arma::vec p;
    //convertPose2PosAngles(pose, p);
    // this->movel(p, 1.2, 1.0, this->cycle);
  }

  void biasFtSensor() { ur_driver->setUrScriptCmd("zero_ftsensor()\n"); }

  void movej(const arma::vec &q, double a, double v, double t, double r=0);

private:

  /** Sends a urscript command to set the robot in control mode 'mode'. */
  void command_mode(const std::string &mode) const;

  void protectiveStop() override;

  void initRobot(const std::string &robot_ip, int reverse_port);

  arma::vec last_joint_pos;

  UrDriver *ur_driver;
  void runUrDriver();
  std::thread ur_driver_thr;
  std::string robot_ip;
  int reverse_port;
  ur_::Semaphore shutdown_sem;

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

} // namespace as64_

#endif  // UR_ROBOT_H
