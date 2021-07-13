#ifndef ROBO_LIB_ROBOT_STATE_PUBLISHER
#define ROBO_LIB_ROBOT_STATE_PUBLISHER

#include <vector>
#include <cstring>
#include <functional>
#include <thread>
#include <chrono>
#include <memory>

#include <kdl_parser/kdl_parser.hpp>

#include <armadillo>

namespace as64_
{

namespace robo_
{

class RobotStatePublisher
{
public:
  RobotStatePublisher(const std::string &robot_desc_param, const std::vector<std::string> &joint_names,
    std::function<arma::vec()> getJointsPosFun, double pub_rate_ms=33);

  ~RobotStatePublisher();

  void start();

  void stop();

  void setPublishCycle(double Ts_ms);

private:

  void publishLoop();

  bool run;

  unsigned long Ts; // in nanosec

  std::thread run_thread;

  std::function<arma::vec()> get_joints_positions_;

  KDL::Tree kdl_tree;

  std::vector<std::string> joint_names;

};

} // namespace robot_

} // namespace as64_


#endif // ROBO_LIB_ROBOT_STATE_PUBLISHER
