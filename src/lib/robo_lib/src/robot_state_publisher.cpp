#include <robo_lib/robot_state_publisher.h>

#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace as64_
{

namespace robo_
{

RobotStatePublisher::RobotStatePublisher(const std::string &robot_desc_param,
  const std::vector<std::string> &joint_names, std::function<arma::vec()> getJointsPosFun, double pub_rate_ms)
{
  run = false;

  setPublishCycle(pub_rate_ms);

  get_joints_positions_ = getJointsPosFun;

  urdf::Model urdf_model;
  if (!urdf_model.initParam(robot_desc_param.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_desc_param + "\"...\n");
  }

  kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree);

  this->joint_names = joint_names;
}

RobotStatePublisher::~RobotStatePublisher()
{
  stop();
}

void RobotStatePublisher::start()
{
  if (run == true) return;
  run = true;
  run_thread = std::thread(&RobotStatePublisher::publishLoop, this);
}

void RobotStatePublisher::stop()
{
  run = false;
  if (run_thread.joinable()) run_thread.join();
}


void RobotStatePublisher::setPublishCycle(double Ts_ms)
{
  this->Ts = Ts_ms*1e6;
}

void RobotStatePublisher::publishLoop()
{
	robot_state_publisher::RobotStatePublisher pub(kdl_tree);

  std::vector<std::string> j_names = this->joint_names;
  int n_joints = j_names.size();

  std::map<std::string, double> j_map;
  for (int i=0; i<n_joints; i++) j_map[j_names[i]] = 0;

  pub.publishFixedTransforms("", true);

  while (run)
  {
    arma::vec joint_positions = get_joints_positions_();
    for (int i=0; i<n_joints; i++) j_map[j_names[i]] = joint_positions(i);
    pub.publishTransforms(j_map, ros::Time::now(), "");

    std::this_thread::sleep_for(std::chrono::nanoseconds(Ts));
  }
}

} // namespace robot_

} // namespace as64_
