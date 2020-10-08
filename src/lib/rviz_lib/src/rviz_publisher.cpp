#include <rviz_lib/rviz_publisher.h>
#include <tf/LinearMath/Vector3.h>

namespace as64_
{

namespace rviz_
{

RvizPublisher::RvizPublisher(const std::string &world_frame, const std::string &publish_topic)
{
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, publish_topic));
  visual_tools_->loadMarkerPub(false);  // create publisher before waiting

  // ROS_INFO("Sleeping 5 seconds before running demo");
  // ros::Duration(5.0).sleep();

  // Clear messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();
}

void RvizPublisher::publishPath(const arma::mat &path, rviz_visual_tools::colors color, rviz_visual_tools::scales scale)
{
  int n_points = path.n_cols;

  std::vector<geometry_msgs::Point> path2(n_points);
  for (int j=0; j<n_points; j++)
  {
    geometry_msgs::Point p;
    p.x = path(0,j);
    p.y = path(1,j);
    p.z = path(2,j);
    path2[j] = p;
  }
  visual_tools_->publishPath(path2, color, scale);
}

void RvizPublisher::publishOrientedPath(const arma::mat &Pos, const arma::mat &Quat, const std::vector<int> orient_frames_ind,
    rviz_visual_tools::colors color, rviz_visual_tools::scales scale)
{
  for (int k=0; k<orient_frames_ind.size(); k++)
    publishFrame(Pos.col(orient_frames_ind[k]), Quat.col(orient_frames_ind[k]), scale);

  publishPath(Pos, color, scale);
}

void RvizPublisher::publishPoint(const arma::vec &pos, rviz_visual_tools::colors color, rviz_visual_tools::scales scale)
{
  std::vector<geometry_msgs::Point> path(1);
  geometry_msgs::Point p;
  p.x = pos(0);
  p.y = pos(1);
  p.z = pos(2);
  path[0] = p;
  visual_tools_->publishSphere(p, color, scale);
}

void RvizPublisher::publishFrame(const arma::vec &pos, const arma::vec &quat, rviz_visual_tools::scales scale)
{
  geometry_msgs::Pose pose;
  pose.position.x = pos(0);
  pose.position.y = pos(1);
  pose.position.z = pos(2);
  pose.orientation.w = quat(0);
  pose.orientation.x = quat(1);
  pose.orientation.y = quat(2);
  pose.orientation.z = quat(3);
  visual_tools_->publishAxis(pose, scale);
}

void RvizPublisher::drawnow()
{
  visual_tools_->trigger();
}

void RvizPublisher::deleteAll()
{
  visual_tools_->deleteAllMarkers();
  drawnow();
}


} // namespace rviz_

} // namespace as64_
