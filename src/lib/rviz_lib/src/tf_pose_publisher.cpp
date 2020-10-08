

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <rviz_lib/tf_pose_publisher.h>

namespace as64_
{

namespace rviz_
{

TfPosePublisher::TfPosePublisher(std::function<arma::vec()> getPose, const std::string &parent_link, const std::string child_link)
{
  get_pose_fun_ = getPose;

  arma::vec temp = get_pose_fun_();
  if (temp.size() != 7) throw std::runtime_error("getPose function must return a vector of length 7!");

  parent_link_ = parent_link;
  child_link_ = child_link;
}

void TfPosePublisher::setPublishRate(unsigned pub_rate_ms)
{
  pub_rate_ms_ = pub_rate_ms;
}

void TfPosePublisher::start(unsigned pub_rate_ms)
{
  pub_rate_ms_ = pub_rate_ms;
  setPublish(true);

  std::thread([this]()
  {
    tf::TransformBroadcaster tf_pub_;
    arma::vec ee_pose;
    while (publish_)
    {
      arma::vec pose = get_pose_fun_();
      arma::vec pos = pose.subvec(0,2);
      arma::vec quat = pose.subvec(3,6);
      tf::Transform target_transform;
      target_transform.setOrigin( tf::Vector3(pos(0), pos(1), pos(2)) );
      target_transform.setRotation( tf::Quaternion(quat(1), quat(2), quat(3), quat(0)) );
      tf_pub_.sendTransform(tf::StampedTransform(target_transform, ros::Time::now(), parent_link_, child_link_));
      std::this_thread::sleep_for(std::chrono::milliseconds(pub_rate_ms_));
    }
  }).detach();
}

void TfPosePublisher::stop()
{
  setPublish(false);
}

void TfPosePublisher::setPublish(bool set)
{
  std::unique_lock<std::mutex> lck(pub_mutex);
  publish_ = set;
}


} // namespace rviz_

} // namespace as64_
