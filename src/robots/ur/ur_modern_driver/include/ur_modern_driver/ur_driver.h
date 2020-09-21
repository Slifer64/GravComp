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

#include "ur_realtime_communication.h"
#include "ur_communication.h"
#include "do_output.h"

#include <ur_modern_driver/utils.h>

class UrDriver
{
private:
	double maximum_time_step_;
	double minimum_payload_;
	double maximum_payload_;
	std::vector<std::string> joint_names_;
	std::string ip_addr_;
	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;
	const unsigned int REVERSE_PORT_;
	int incoming_sockfd_;
	int new_sockfd_;
	bool reverse_connected_;
	double servoj_time_;
	bool executing_traj_;
	double firmware_version_;
	double servoj_lookahead_time_;
	double servoj_gain_;

  ur_::Semaphore rt_msg_sem;
  ur_::Semaphore msg_sem;

  std::vector<double> joint_offsets_;

  bool keep_alive_;

  std::thread rt_read_thread_;
  std::thread mb_read_thread_;

  ur_::Timer timer;

  double t; // current timestamp
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  std::vector<double> effort;
  std::vector<double> tcp_wrench;
  std::vector<double> tcp_pos;
  std::vector<double> tcp_quat;
  std::vector<double> tcp_vel;

  std::vector<double> joint_target_vel;

  as64_::ur_::MtxVar<std::string> ur_script_cmd;

public:

  ur_::Timer global_timer;

  arma::vec joint_vel_cmd;
	arma::vec joint_pos_cmd;

  int n_data;
  arma::rowvec time_data;
	arma::mat joint_pos_data;
  arma::mat joint_vel_data;
  arma::mat joint_target_vel_data;
  arma::mat joint_vel_cmd_data;
	arma::mat joint_pos_cmd_data;

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

  std::vector<double> getJointTargetVel() const { return joint_target_vel; }

	UrRealtimeCommunication *rt_interface_;
	UrCommunication* sec_interface_;

  ur_::Semaphore update_sem;

	UrDriver(std::string host, unsigned int reverse_port = 50001, double servoj_time = 0.008, unsigned int safety_count_max =
					12, double max_time_step = 0.08, double min_payload = 0.,
			double max_payload = 1., double servoj_lookahead_time=0.03, double servoj_gain=300.);
  ~UrDriver();
	bool start();
	void halt();

	void setSpeed(double q0, double q1, double q2, double q3, double q4,
			double q5, double acc = 100.);

	bool doTraj(std::vector<double> inp_timestamps,
			std::vector<std::vector<double> > inp_positions,
			std::vector<std::vector<double> > inp_velocities);
	void servoj(std::vector<double> positions, int keepalive = 1);

	void stopTraj();

	bool uploadProg();
	bool openServo();
	void closeServo(std::vector<double> positions);

	std::vector<double> interp_cubic(double t, double T,
			std::vector<double> p0_pos, std::vector<double> p1_pos,
			std::vector<double> p0_vel, std::vector<double> p1_vel);

	std::vector<std::string> getJointNames();
	void setJointNames(std::vector<std::string> jn);
	void setToolVoltage(unsigned int v);
	void setFlag(unsigned int n, bool b);
	void setDigitalOut(unsigned int n, bool b);
	void setAnalogOut(unsigned int n, double f);
	bool setPayload(double m);

	void setMinPayload(double m);
	void setMaxPayload(double m);
	void setServojTime(double t);
	void setServojLookahead(double t);
	void setServojGain(double g);

  void readRTMsg();
  void readMbMsg();

  void setUrScriptCmd(const std::string &cmd) { ur_script_cmd.set(cmd); }

  double getServojTime() const { return servoj_time_; }

};

#endif /* UR_DRIVER_H_ */
