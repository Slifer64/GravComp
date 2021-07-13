/*
 * ur_driver
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_DRIVER_H_
#define UR_DRIVER_H_

#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>
#include <condition_variable>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <armadillo>

#include <ur_modern_driver/ur_realtime_communication.h>
#include <ur_modern_driver/ur_communication.h>
#include <ur_modern_driver/do_output.h>
#include <ur_modern_driver/socket_com.h>

#include <ur_modern_driver/utils.h>

class UrDriver
{

enum State
{
  JOINT_POS_CTRL = 1,
	JOINT_VEL_CTRL = 2,
	CART_POS_CTRL = 3,
	CART_VEL_CTRL = 4,
  FREEDRIVE = 5,
  IDLE_MODE = 6,
	BIAS_FT_SENSOR = 7,
	TERMINATE = 8,
};

public:

  UrDriver(const std::string &host_ip, const std::string &robot_ip, unsigned reverse_port = 8080);
  ~UrDriver();
	bool start();
	void halt();

  bool isDisconnected() const { return !reverse_connected_; }
  bool isEmergencyStopped() const { return sec_interface_->robot_state_.isProtectiveStopped(); }
  bool isRobotConnected() const { return sec_interface_->robot_state_.isRobotConnected(); }
  bool isProgramRunning() const { return sec_interface_->robot_state_.isProgramRunning(); }
  bool isProtectiveStopped() const { return sec_interface_->robot_state_.isEmergencyStopped(); }

  double getControllerTime() const { return t; }
  std::vector<double> getJointPos() const { return joint_pos; }
  std::vector<double> getJointVel() const { return joint_vel; }
  std::vector<double> getEffort() const { return effort; }
  std::vector<double> getTcpWrench() const { return tcp_wrench; }
  std::vector<double> getTcpPos() const { return tcp_pos; }
  std::vector<double> getTcpQuat() const { return tcp_quat; }
  std::vector<double> getTcpVel() const { return tcp_vel; }
  std::vector<double> getJointTorque() const { return joint_torq; }

  std::vector<double> getJointTargetVel() const { return joint_target_vel; }

	UrRealtimeCommunication *rt_interface_;
	UrCommunication* sec_interface_;

  ur_::Semaphore update_sem;

  void readRTMsg();
  void readMbMsg();

  void setUrScriptCmd(const std::string &cmd) { ur_script_cmd.set(cmd); }

  void startReverseCom();
  void stopReverseCom();

  void setJointsPosition(const arma::vec &j_pos)
  { writeCommand(JOINT_POS_CTRL, j_pos, 0, 0); }

  void setJointsVelocity(const arma::vec &j_vel, double a)
  { writeCommand(JOINT_VEL_CTRL, j_vel, 0, a); }

  // pose: [pos; k*theta]
  void setTaskPose(const arma::vec &pose)
  { writeCommand(CART_POS_CTRL, pose, 0, 0); }

  void setTaskVelocity(const arma::vec &task_vel, double a)
  { writeCommand(CART_VEL_CTRL, task_vel, 0, a); }

  void freedrive_mode();

  void idle_mode();

  void terminate();

  void biasFtSensor();

private:

  std::string loadUrDriverProgram();

  void readRobotStateThread();
  void unpackState(char *buff, int size);
  void unpackVector(char *buff, int size, int &i1, int &i2, std::vector<double> &unpack_to);

  char *writeDouble(char *buf, double val);

  char *writeInt(char *buf, int val);

  void writeCommand(int state, const arma::vec &cmd_, double vel, double accel);

  std::vector<std::string> joint_names_;
  std::string ip_addr_;
  const int MULT_JOINTSTATE_ = 1000000;
  int incoming_sockfd_;
  int new_sockfd_;
  bool reverse_connected_;
  double firmware_version_;

  const std::string host_ip_;
  const std::string robot_ip_;
  const unsigned int REVERSE_PORT_;

  ur_::Semaphore rt_msg_sem;
  ur_::Semaphore msg_sem;

  bool keep_alive_;

  std::thread rt_read_thread_;
  std::thread mb_read_thread_;

  std::thread read_state_thr;

  ur_::Timer timer;

  double t; // current timestamp
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  std::vector<double> joint_torq;
  std::vector<double> effort;
  std::vector<double> tcp_wrench;
  std::vector<double> tcp_pos;
  std::vector<double> tcp_quat;
  std::vector<double> tcp_vel;

  std::vector<double> joint_target_vel;

  ur_::MtxVar<std::string> ur_script_cmd;

  std::string ur_driver_program_;
};

#endif /* UR_DRIVER_H_ */
