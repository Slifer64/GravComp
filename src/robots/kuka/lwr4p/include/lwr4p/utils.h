#ifndef LWR4P_UTILS_H
#define LWR4P_UTILS_H

#include <Eigen/Dense>
#include <armadillo>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace as64_
{

namespace lwr4p_
{

enum Mode
{
  IDLE = 0,
  FREEDRIVE = 1,
  JOINT_POS_CONTROL = 2,
  JOINT_VEL_CONTROL = 3,
  JOINT_TORQUE_CONTROL = 4,
  CART_VEL_CONTROL = 5,
  CART_IMPEDANCE_CONTROL = 6,
  PROTECTIVE_STOP = 7
};

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);

arma::vec rotm2quat(const arma::mat &rotm);

std::vector<arma::vec> get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

void print_err_msg(const std::string &msg);
void print_info_msg(const std::string &msg);
void print_warn_msg(const std::string &msg);

class Timer
{
public:
    Timer()
    {
      clock_gettime(CLOCK_REALTIME, &beg_);
    }

    long int elapsedNanoSec()
    {
        clock_gettime(CLOCK_REALTIME, &end_);
        return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
    }

    double elapsedMicroSec()
    {
        return elapsedNanoSec()/1000.0;
    }

    double elapsedMilliSec()
    {
        return elapsedNanoSec()/1000000.0;
    }

    double elapsedSec()
    {
        return elapsedNanoSec()/1000000000.0;
    }

    void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};

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

}; // namespace LWR4P_

}; // namespace as64_

#endif // LWR4P_UTILS_H
