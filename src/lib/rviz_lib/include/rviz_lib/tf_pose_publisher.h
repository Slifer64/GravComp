
#ifndef AS64_TF_POSE_PUBLISHER_H
#define AS64_TF_POSE_PUBLISHER_H

#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#include <armadillo>

namespace as64_
{

namespace rviz_
{

class TfPosePublisher
{
public:

  TfPosePublisher(std::function<arma::vec()> getPose, const std::string &parent_link, const std::string child_link="pose");

  void start(unsigned pub_rate_ms = 200);

  void stop();

  void setPublishRate(unsigned pub_rate_ms);

private:

  void setPublish(bool set);

  unsigned pub_rate_ms_;

  std::function<arma::vec()> get_pose_fun_; // return the pose as [pos; quat]

  bool publish_;
  std::mutex pub_mutex;

  std::string parent_link_;
  std::string child_link_;
};


} // namespace rviz_

} // namespace as64_

#endif // AS64_TF_POSE_PUBLISHER_H
