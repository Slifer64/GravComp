#ifndef UR_ROS_WRAPPER_H
#define UR_ROS_WRAPPER_H

#include "ur_modern_driver/ur_driver.h"
#include "ur_modern_driver/ur_hardware_interface.h"
#include "ur_modern_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace ur_
{

class RosWrapper
{
protected:
  bool keep_alive;
	UrDriver *robot_;
	Semaphore rt_msg_sem; //std::condition_variable rt_msg_cond_;
  Semaphore msg_sem; //std::condition_variable msg_cond_;
	ros::NodeHandle nh_;
	actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
	bool has_goal_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;
	ros::Subscriber speed_sub_;
	ros::Subscriber urscript_sub_;
	ros::ServiceServer io_srv_;
	ros::ServiceServer payload_srv_;
	std::thread rt_publish_thread_;
	std::thread mb_publish_thread_;
	double io_flag_delay_;
	double max_velocity_;
	std::vector<double> joint_offsets_;
    std::string base_frame_;
    std::string tool_frame_;
	bool use_ros_control_;
	std::thread* ros_control_thread_;
	boost::shared_ptr<ros_control_ur::UrHardwareInterface> hardware_interface_;
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	ros::AsyncSpinner spinner;

public:

	RosWrapper(const std::string &robot_ip_address="localhost", int reverse_port=50001);

	~RosWrapper();

	void init(std::string host, int reverse_port);

	void halt();

private:
	void trajThread(std::vector<double> timestamps,
			std::vector<std::vector<double> > positions,
			std::vector<std::vector<double> > velocities);

	void goalCB(
			actionlib::ServerGoalHandle<
					control_msgs::FollowJointTrajectoryAction> gh);

	void cancelCB(
			actionlib::ServerGoalHandle<
					control_msgs::FollowJointTrajectoryAction> gh);

	bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& resp);

	bool setPayload(ur_msgs::SetPayloadRequest& req,
			ur_msgs::SetPayloadResponse& resp);

	bool validateJointNames();

	void reorder_traj_joints(trajectory_msgs::JointTrajectory& traj);

	bool has_velocities();

	bool has_positions();

	bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);

	bool has_limited_velocities();

	bool traj_is_finite();

	void speedInterface(const trajectory_msgs::JointTrajectory::Ptr& msg);

	void urscriptInterface(const std_msgs::String::ConstPtr& msg);

	void rosControlLoop();

	void publishRTMsg();

	void publishMbMsg();

}; // class RosWrapper

} // namespace ur_

#endif // UR_ROS_WRAPPER_H
