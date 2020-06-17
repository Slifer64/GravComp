#ifndef AS64_JOINT_STATE_PUBLISHER
#define AS64_JOINT_STATE_PUBLISHER

#include <vector>
#include <cstring>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace as64_
{

namespace robo_
{

class JointStatePublisher
{
public:
  JointStatePublisher();

  ~JointStatePublisher();

  void start();

  void stop();

  void addFun(void (*fun)(sensor_msgs::JointState &));

  template<class T>
  void addFun(void (T::*fun)(sensor_msgs::JointState &), T *obj_ptr)
  {
    std::unique_lock<std::mutex> lck(mtx);
    add_joint_state_funs.push_back( std::bind(fun, obj_ptr, std::placeholders::_1) );
  }

  void setPublishCycle(double Ts);
  void setPublishTopic(const std::string &pub_topic);

private:

  void publishLoop();

  bool run;

  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;

  std::string pub_topic;
  unsigned long Ts; // in nanosec

  std::thread run_thread;
  std::mutex mtx;

  std::vector<std::function<void(sensor_msgs::JointState &)>> add_joint_state_funs;

};

} // namespace robo_

} // namespace as64_

#endif // AS64_JOINT_STATE_PUBLISHER
