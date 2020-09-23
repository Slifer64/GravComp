#ifndef AS64_TF_POSE_PUBLISHER_H
#define AS64_TF_POSE_PUBLISHER_H

#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <armadillo>

class TfPosePublisher
{
public:
  TfPosePublisher(std::function<arma::vec()> getPos, std::function<arma::vec()> getQuat, const std::string &parent_link, const std::string child_link="pose")
  {
    get_pos = getPos;
    get_quat = getQuat;

    parent_link_ = parent_link;
    child_link_ = child_link;
  }

  void start(unsigned pub_rate_ms = 200)
  {
    setPublish(true);

    std::thread([this, pub_rate_ms]()
    {
      tf::TransformBroadcaster tf_pub_;
      arma::vec ee_pose;
      while (publish_)
      {
        arma::vec pos = get_pos();
        arma::vec quat = get_quat();
        tf::Transform target_transform;
        target_transform.setOrigin( tf::Vector3(pos(0), pos(1), pos(2)) );
        target_transform.setRotation( tf::Quaternion(quat(1), quat(2), quat(3), quat(0)) );
        tf_pub_.sendTransform(tf::StampedTransform(target_transform, ros::Time::now(), parent_link_, child_link_));
        std::this_thread::sleep_for(std::chrono::milliseconds(pub_rate_ms));
      }
    }).detach();
  }

  void stop() { setPublish(false); }

private:

  void setPublish(bool set)
  {
    std::unique_lock<std::mutex> lck(pub_mutex);
    publish_ = set;
  }

  std::function<arma::vec()> get_pos;
  std::function<arma::vec()> get_quat;

  bool publish_;
  std::mutex pub_mutex;

  std::string parent_link_;
  std::string child_link_;
};

#endif // AS64_TF_POSE_PUBLISHER_H
