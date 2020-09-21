#ifndef UR_UTILS_H
#define UR_UTILS_H

#include <Eigen/Dense>
#include <armadillo>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace as64_
{

namespace ur_
{

enum Mode
{
  NORMAL = 1,
  FREEDRIVE = 2,
  STOPPED = 3,
  PROTECTIVE_STOP = 4
};

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);

arma::vec rotm2quat(const arma::mat &rotm);

arma::mat quat2rotm(const arma::vec &quat);

std::vector<arma::vec> get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

void print_err_msg(const std::string &msg);
void print_info_msg(const std::string &msg);
void print_warn_msg(const std::string &msg);

class WrenchReader
{
public:
  WrenchReader(const std::string &read_wrench_topic)
  {
    sub = nh.subscribe(read_wrench_topic.c_str(), 1, &WrenchReader::readWrenchCallback, this);

    wrench = arma::vec().zeros(6);
  }

  arma::vec getWrench()
  {
    ros::spinOnce(); // update wrench from "readWrenchCallback"
    return wrench;
  }

private:

  arma::vec wrench;
  ros::Subscriber sub;
  ros::NodeHandle nh;

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr)
  {
    wrench(0) = wrench_ptr->wrench.force.x;
    wrench(1) = wrench_ptr->wrench.force.y;
    wrench(2) = wrench_ptr->wrench.force.z;
    wrench(3) = wrench_ptr->wrench.torque.x;
    wrench(4) = wrench_ptr->wrench.torque.y;
    wrench(5) = wrench_ptr->wrench.torque.z;
  }
};

}; // namespace ur_

}; // namespace as64_

#endif // UR_UTILS_H
