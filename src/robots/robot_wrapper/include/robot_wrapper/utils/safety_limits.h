#ifndef AS64_ROBOT_WRAPPER_SAFETY_LIMITS_H
#define AS64_ROBOT_WRAPPER_SAFETY_LIMITS_H

#include <exception>
#include <cstring>
#include <armadillo>
#include <ros/package.h>
#include <robot_wrapper/utils/xml_parser.h>

namespace rw_

{

class SafetyLimits
{
public:
  SafetyLimits()
  {
    XmlParser parser(ros::package::getPath("robot_wrapper") + "/config/safety_limits.yaml");
    if (!parser.getParam("tool_vel_lim",tool_vel_lim)) tool_vel_lim = 0.8;
    if (!parser.getParam("tool_rot_vel_lim",tool_rot_vel_lim)) tool_rot_vel_lim = 3.14;
    if (!parser.getParam("joint_vel_lim",joint_vel_lim)) joint_vel_lim = 3.14;
  }

  bool assertToolVelLim(const arma::vec &tool_vel, std::string *msg=0)
  {
    if (!assertToolTransVelLim(tool_vel.subvec(0,2)) && assertToolRotVelLim(tool_vel.subvec(3,5)))
    {
      if (msg) *msg = "Tool vel limit exceeded: v_trans = " + std::to_string(arma::norm(tool_vel.subvec(0,2)))
                      + " , v_rot = " + std::to_string(arma::norm(tool_vel.subvec(3,5)));
      return false;
    }
    return true;
  }

  bool assertToolTransVelLim(const arma::vec &tool_vel)
  {
    return (arma::norm(tool_vel) < tool_vel_lim); // throw std::runtime_error("[ALERT]: Tool translational velocity limit exceeded!\n");
  }

  bool assertToolRotVelLim(const arma::vec &tool_rot_vel)
  {
    return (arma::norm(tool_rot_vel) < tool_rot_vel_lim); // throw std::runtime_error("[ALERT]: Tool rotational velocity limit exceeded!\n");
  }

  bool assertJointVelLim(const arma::vec &joint_vel)
  {
    for (int i=0; i<joint_vel.size(); i++)
    {
      if (std::fabs(joint_vel(i)) > joint_vel_lim) return false; // throw std::runtime_error("[ALERT]: Joint velocity limit exceeded!\n");
    }
    return true;

  }

private:
  double tool_vel_lim; // m/s
  double tool_rot_vel_lim; // rad/s
  double joint_vel_lim; // rad/s
};

} // namespace rw_

#endif // AS64_ROBOT_WRAPPER_SAFETY_LIMITS_H
