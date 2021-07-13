#include <robot_wrapper/utils/publish_state_interface.h>
#include <robot_wrapper/utils/xml_parser.h>

#include <ros/package.h>

namespace rw_
{

#define PublishStateInterface_fun_ std::string("[PublishStateInterface::") + __func__ + "]: "


PublishStateInterface::PublishStateInterface()
{

}

PublishStateInterface::~PublishStateInterface()
{
  rState_pub->stop();
}

void PublishStateInterface::initPublishStateInterface(const std::string &robot_desc_param,
  const std::vector<std::string> &joint_names, std::function<arma::vec()> getJointsPosFun)
{
  rState_pub.reset(new robo_::RobotStatePublisher(robot_desc_param, joint_names, getJointsPosFun));

  std::string params_filename = ros::package::getPath("robot_wrapper") + "/config/params.yaml";
  rw_::XmlParser parser(params_filename);

  std::string load_fail_msg = PublishStateInterface_fun_ + "Failed to load param ";

  // check whether to publish joint states
  bool pub_robot_state_flag = false;
  if (parser.getParam("pub_robot_state_flag",pub_robot_state_flag))
  {
    double pub_rate_ms;
    if (!parser.getParam("pub_rate_ms",pub_rate_ms)) pub_rate_ms = 33;
    rState_pub->setPublishCycle(pub_rate_ms);
    rState_pub->start();
  }
}

} // namespace rw_
