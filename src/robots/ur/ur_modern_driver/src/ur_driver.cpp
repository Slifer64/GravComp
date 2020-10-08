/*
 * ur_driver.cpp
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

#include <ur_modern_driver/ur_driver.h>
#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/socket_com.h>

#include <ros/package.h>

using namespace ur_;

#define UrDriver_fun_ std::string("[UrDriver::") + __func__ + "]: "

UrDriver::UrDriver(std::string host, unsigned int reverse_port, double servoj_time, double max_time_step,
	  double min_payload, double max_payload, double servoj_lookahead_time, double servoj_gain) :
		REVERSE_PORT_(reverse_port), maximum_time_step_(max_time_step), minimum_payload_(min_payload),
		maximum_payload_(max_payload), servoj_time_(servoj_time), servoj_lookahead_time_(servoj_lookahead_time), servoj_gain_(servoj_gain)
{
  t = 0;
  joint_pos = {6, 0.0};
  joint_vel = {6, 0.0};
  effort = {6, 0.0};
  tcp_wrench = {6, 0.0};
  tcp_pos = {3, 0.0};
  tcp_quat = {4, 0.0};
  tcp_vel = {6, 0.0};
  joint_target_vel = {6, 0.0};

  if((reverse_port <= 0) or (reverse_port >= 65535))
  {
    print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 50001");
    reverse_port = 50001;
  }

  // joint_offsets_ = {6, 0.0};

	firmware_version_ = 0;
	reverse_connected_ = false;
	executing_traj_ = false;
	rt_interface_ = new UrRealtimeCommunication(rt_msg_sem, host);
	new_sockfd_ = -1;
	sec_interface_ = new UrCommunication(msg_sem, host);

	incoming_sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);
	if (incoming_sockfd_ < 0) print_fatal("ERROR opening socket for reverse communication");

	keep_alive_ = true;
}

UrDriver::~UrDriver()
{
	stopReverseCom();
  delete rt_interface_;
  delete sec_interface_;
}

void UrDriver::startReverseCom()
{
	std::string filename = ros::package::getPath("ur_modern_driver") + "/config/ur_driver_program.txt";
	std::string program_;
	ur_::readFile(filename, program_);

	struct sockaddr_in serv_addr;
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(REVERSE_PORT_);

	// com_::setNoDelay(incoming_sockfd_, true);
	com_::setReuseAddr(incoming_sockfd_, true);

	if (bind(incoming_sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		throw std::runtime_error("ERROR on binding socket for reverse communication");

	if ( listen(incoming_sockfd_, 1) < 0 ) // listen(server_fid, SOMAXCONN)
		throw std::runtime_error(UrDriver_fun_ + "Error on \"listen()\"...\n");

	com_::setNonBlocking(incoming_sockfd_, true);

	setUrScriptCmd(program_);

	struct timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	int err_code;
	com_::WaitResult res = com_::waitForRead(incoming_sockfd_, timeout, &err_code);

	print_debug(UrDriver_fun_ + "Waiting to accept client...\n");
	struct sockaddr_in cli_addr;
	socklen_t clilen;
	clilen = sizeof(cli_addr);
	new_sockfd_ = accept(incoming_sockfd_, (struct sockaddr *) &cli_addr, &clilen);

	if (res != com_::READY)
	{
		if (res == com_::ERROR) throw std::runtime_error(UrDriver_fun_ + com_::getErrMsg(err_code));
		if (res == com_::TIMEOUT) throw std::runtime_error(UrDriver_fun_ + "Timeout reached on waiting to accept client...\n");
	}

	if (new_sockfd_ < 0) throw std::runtime_error(UrDriver_fun_ + "ERROR on accepting reverse communication");

	print_debug(UrDriver_fun_ + "Listening on " + com_::getLocalIp(new_sockfd_) + ":" + std::to_string(com_::getLocalPort(new_sockfd_)) + "\n");

	reverse_connected_ = true;
}

void UrDriver::stopReverseCom()
{
	if (reverse_connected_)
	{
		rt_interface_->addCommandToQueue("stopj(10)\n");
		reverse_connected_ = false;
		close(new_sockfd_);
	}
}

bool UrDriver::start()
{
	if (!sec_interface_->start()) return false;
	firmware_version_ = sec_interface_->robot_state_.getVersion();
	rt_interface_->robot_state_.setVersion(firmware_version_);
	if (!rt_interface_->start()) return false;
	ip_addr_ = rt_interface_->getLocalIp();
	std::cout << ("Listening on " + ip_addr_ + ":" + std::to_string(REVERSE_PORT_) + "\n");
	print_debug("Listening on " + ip_addr_ + ":" + std::to_string(REVERSE_PORT_) + "\n");

	// launch thread for reading the robot's state
  rt_read_thread_ = std::thread(std::bind(&UrDriver::readRTMsg, this));
  // mb_read_thread_ = std::thread(std::bind(&UrDriver::readMbMsg, this));

  int err_code = ur_::makeThreadRT(rt_read_thread_);
  if (err_code) ur_::PRINT_WARNING_MSG("[UrDriver::start]: Failed to set thread priority! Reason:\n" + ur_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG("[UrDriver::start]: Set thread priority successfully!\n", std::cerr);

	startReverseCom();

  return true;
}

void UrDriver::halt()
{
  keep_alive_ = false;
	sec_interface_->halt();
	rt_interface_->halt();
	close(incoming_sockfd_);

  if (rt_read_thread_.joinable()) rt_read_thread_.join();
}

void UrDriver::setToolVoltage(unsigned int v)
{
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", v);
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}

void UrDriver::setFlag(unsigned int n, bool b)
{
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_flag(%d, %s)\nend\n", n,
			b ? "True" : "False");
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}

void UrDriver::setDigitalOut(unsigned int n, bool b)
{
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_digital_out(%d, %s)\nend\n", n,
				b ? "True" : "False");
    } else if (n > 15) {
        sprintf(buf,
                "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
                n - 16, b ? "True" : "False");
	} else if (n > 7) {
        sprintf(buf, "sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
				n - 8, b ? "True" : "False");

	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
				n, b ? "True" : "False");

	}
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);

}

void UrDriver::setAnalogOut(unsigned int n, double f)
{
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_analog_out(%d, %1.4f)\nend\n", n, f);
	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", n, f);
	}

	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}

bool UrDriver::setPayload(double m)
{
	if ((m < maximum_payload_) && (m > minimum_payload_)) {
		char buf[256];
		sprintf(buf, "sec setOut():\n\tset_payload(%1.3f)\nend\n", m);
		rt_interface_->addCommandToQueue(buf);
		print_debug(buf);
		return true;
	} else
		return false;
}

void UrDriver::setMinPayload(double m)
{
	if (m > 0) minimum_payload_ = m;
	else minimum_payload_ = 0;
}

void UrDriver::setMaxPayload(double m)
{
	maximum_payload_ = m;
}

void UrDriver::setServojTime(double t)
{
	if (t > 0.008) servoj_time_ = t;
	else servoj_time_ = 0.008;
}

void UrDriver::setServojLookahead(double t)
{
	if (t > 0.03)
	{
		if (t < 0.2) servoj_lookahead_time_ = t;
		else servoj_lookahead_time_ = 0.2;
	}
	else
	{
		servoj_lookahead_time_ = 0.03;
	}
}

void UrDriver::setServojGain(double g)
{
	if (g > 100) {
			if (g < 2000) {
				servoj_gain_ = g;
			} else {
				servoj_gain_ = 2000;
			}
		} else {
			servoj_gain_ = 100;
		}
}

void UrDriver::readRTMsg()
{
  // unsigned long ctrl_cycle = servoj_time_*1e9;
	// unsigned long ctrl_cycle = 0.001*1e9;
  // timer.start();

  try
  {

  while (keep_alive_)
  {
    rt_msg_sem.wait(); // wait for new data to arrive...

    t = rt_interface_->robot_state_.getControllerTimer();

    joint_pos = rt_interface_->robot_state_.getQActual();
    // for (unsigned int i = 0; i < joint_pos.size(); i++) joint_pos[i] += joint_offsets_[i];
    joint_vel = rt_interface_->robot_state_.getQdActual();
    effort = rt_interface_->robot_state_.getIActual();
    tcp_wrench = rt_interface_->robot_state_.getTcpWrench();

    joint_target_vel = rt_interface_->robot_state_.getQdTarget();

    // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz),
    // where rx, ry and rz is a rotation vector representation of the tool orientation
    std::vector<double> tcp_pose = rt_interface_->robot_state_.getToolVectorActual();

    tcp_pos = {tcp_pose[0], tcp_pose[1], tcp_pose[2]};

    //tcp orientation as unit quaternion
    double rx = tcp_pose[3];
    double ry = tcp_pose[4];
    double rz = tcp_pose[5];
    double angle = std::sqrt(std::pow(rx,2) + std::pow(ry,2) + std::pow(rz,2));
    tcp_quat.resize(4);
    if (angle < 1e-16) tcp_quat = {1, 0, 0, 0};
    else
    {
      tcp_quat[0] = std::cos(angle/2);
      double sin_a = std::sin(angle/2);
      tcp_quat[1] = sin_a*rx/angle;
      tcp_quat[2] = sin_a*ry/angle;
      tcp_quat[3] = sin_a*rz/angle;
    }

    // tool velocity
    tcp_vel = rt_interface_->robot_state_.getTcpSpeedActual();

    if (!ur_script_cmd().empty())
    {
      rt_interface_->addCommandToQueue(ur_script_cmd.get());
      ur_script_cmd.set(""); // clear previous command
    }

    update_sem.notify();
  }

  }
  catch(std::exception &e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
  }

}

void UrDriver::readMbMsg()
{
  bool warned = false;

  while (keep_alive_)
  {
//    ur_msgs::IOStates io_msg;
//    msg_sem.wait(); // wait for new data to arrive...
//    int i_max = 10;
//    if (sec_interface_->robot_state_.getVersion() > 3.0)
//      i_max = 18; // From version 3.0, there are up to 18 inputs and outputs
//    for (unsigned int i = 0; i < i_max; i++)
//    {
//      ur_msgs::Digital digi;
//      digi.pin = i;
//      digi.state = ((sec_interface_->robot_state_.getDigitalInputBits() & (1 << i)) >> i);
//      io_msg.digital_in_states.push_back(digi);
//      digi.state = ((sec_interface_->robot_state_.getDigitalOutputBits() & (1 << i)) >> i);
//      io_msg.digital_out_states.push_back(digi);
//    }
//    ur_msgs::Analog ana;
//    ana.pin = 0;
//    ana.state = sec_interface_->robot_state_.getAnalogInput0();
//    io_msg.analog_in_states.push_back(ana);
//    ana.pin = 1;
//    ana.state = sec_interface_->robot_state_.getAnalogInput1();
//    io_msg.analog_in_states.push_back(ana);
//
//    ana.pin = 0;
//    ana.state = sec_interface_->robot_state_.getAnalogOutput0();
//    io_msg.analog_out_states.push_back(ana);
//    ana.pin = 1;
//    ana.state = sec_interface_->robot_state_.getAnalogOutput1();
//    io_msg.analog_out_states.push_back(ana);

    if (sec_interface_->robot_state_.isEmergencyStopped() or sec_interface_->robot_state_.isProtectiveStopped())
    {
      if (sec_interface_->robot_state_.isEmergencyStopped() and !warned) print_error("Emergency stop pressed!");
      else if (sec_interface_->robot_state_.isProtectiveStopped() and !warned) print_error("Robot is protective stopped!");
      warned = true;
    }
    else warned = false;

    sec_interface_->robot_state_.finishedReading();
  }
}

char *UrDriver::writeDouble(char *buf, double val)
{
	return writeInt(buf, (int) (val * MULT_JOINTSTATE_));
}

char *UrDriver::writeInt(char *buf, int val)
{
	val = htonl(val);
	*buf++ = val & 0xff;
	*buf++ = (val >> 8) & 0xff;
	*buf++ = (val >> 16) & 0xff;
	*buf++ = (val >> 24) & 0xff;

	return buf;
}

void UrDriver::writeCommand(int state, const arma::vec &cmd_, double vel, double accel)
{
	const int len = 4*9;
	char buff[len];
	// memset(buff, 0, len);

	char *ptr = buff;
	ptr = writeInt(ptr, state);
	for (int i=0; i<6; i++) ptr = writeDouble(ptr, cmd_(i));
	ptr = writeDouble(ptr, vel);
	ptr = writeDouble(ptr, accel);

	int n_bytes = ur_::com_::write(new_sockfd_, buff, len, true);
	if (n_bytes != len) throw std::runtime_error("Error: sent bytes=" + std::to_string(n_bytes) + ", bytes to send=" + std::to_string(len) + "\n");
}

void UrDriver::freedrive_mode()
{
	const int len = 4;
	char buff[len];
	writeInt(buff, FREEDRIVE);

	int n_bytes = ur_::com_::write(new_sockfd_, buff, len, true);
	if (n_bytes != len) throw std::runtime_error("Error: sent bytes=" + std::to_string(n_bytes) + ", bytes to send=" + std::to_string(len) + "\n");
}

void UrDriver::biasFtSensor()
{
	const int len = 4;
	char buff[len];
	writeInt(buff, BIAS_FT_SENSOR);

	int n_bytes = ur_::com_::write(new_sockfd_, buff, len, true);
	if (n_bytes != len) throw std::runtime_error("Error: sent bytes=" + std::to_string(n_bytes) + ", bytes to send=" + std::to_string(len) + "\n");
}
