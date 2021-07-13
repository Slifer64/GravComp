#ifndef AS64_ROBOT_WRAPPER_PUBLISH_STATE_INTERFACE_H
#define AS64_ROBOT_WRAPPER_PUBLISH_STATE_INTERFACE_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <armadillo>

#include <robo_lib/robot_state_publisher.h>

using namespace as64_;

namespace rw_
{

class PublishStateInterface
{
public:
  PublishStateInterface();
  ~PublishStateInterface();

  void initPublishStateInterface(const std::string &robot_desc_param,
    const std::vector<std::string> &joint_names, std::function<arma::vec()> getJointsPosFun);

protected:
  std::unique_ptr<robo_::RobotStatePublisher> rState_pub;
};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_PUBLISH_STATE_INTERFACE_H
