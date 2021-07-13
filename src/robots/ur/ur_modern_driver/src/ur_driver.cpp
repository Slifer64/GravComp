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

#include <io_lib/file_io.h>

using namespace ur_;

#define USE_readRTMsg 0
// #define LOG_TIMES
#define TIME_BUFF_SIZE 100000

#define UrDriver_fun_ std::string("[UrDriver::") + __func__ + "]: "

UrDriver::UrDriver(const std::string &host_ip, const std::string &robot_ip, unsigned reverse_port):
REVERSE_PORT_(reverse_port), host_ip_(host_ip), robot_ip_(robot_ip)
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
	rt_interface_ = new UrRealtimeCommunication(rt_msg_sem, robot_ip_);
	new_sockfd_ = -1;
	sec_interface_ = new UrCommunication(msg_sem, robot_ip_);

  ur_driver_program_ = loadUrDriverProgram();

	keep_alive_ = true;
}

UrDriver::~UrDriver()
{
  PRINT_WARNING_MSG("\n\n[UrDriver::~UrDriver]: Destructing!!!\n\n");
	halt();
  delete rt_interface_;
  delete sec_interface_;
}

std::string UrDriver::loadUrDriverProgram()
{
  std::string filename = ros::package::getPath("ur_modern_driver") + "/config/ur_driver_program.txt";
  std::string program_;
  ur_::readFile(filename, program_);

  std::vector<std::string> placeholder_ = {"<HOST_IP>", "<REVERSE_PORT>", "<MULT>"};
  std::vector<std::string> value_ = {"\""+host_ip_+"\"", std::to_string(REVERSE_PORT_), std::to_string(MULT_JOINTSTATE_)};

  auto str_rep = [&program_](const std::string &ph_, const std::string &val)
  {
    int i = program_.find(ph_);
    if (i==std::string::npos) throw std::runtime_error(UrDriver_fun_ + "Failed to find \"" + ph_ + "\". Is the ur_driver program corrupted?");
    program_ = program_.substr(0, i) + val + program_.substr(i+ph_.length());
  };

  for (int i=0; i<placeholder_.size(); i++) str_rep(placeholder_[i], value_[i]);

  return program_;
}

void UrDriver::startReverseCom()
{
	incoming_sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);
	// com_::setNoDelay(incoming_sockfd_, true);
	com_::setReuseAddr(incoming_sockfd_, true);
  com_::bind(incoming_sockfd_, REVERSE_PORT_, host_ip_); // com_::bind(incoming_sockfd_, REVERSE_PORT_, "0.0.0.0");
	com_::listen(incoming_sockfd_, 1);
	com_::setNonBlocking(incoming_sockfd_, true);

	//setUrScriptCmd(program_);
	rt_interface_->addCommandToQueue(ur_driver_program_);

	print_debug(UrDriver_fun_ + "Waiting to accept client...\n");
	new_sockfd_ = com_::acceptClient(incoming_sockfd_, com_::Timeout(10,0));

	print_debug(UrDriver_fun_ + "Listening on " + com_::getLocalIp(new_sockfd_) + ":" + std::to_string(com_::getLocalPort(new_sockfd_)) + "\n");

  reverse_connected_ = true;

	if (!USE_readRTMsg) readRobotStateThread();
}

void UrDriver::stopReverseCom()
{
  keep_alive_ = false;
  if (read_state_thr.joinable()) read_state_thr.join();
}

void UrDriver::readRobotStateThread()
{

	read_state_thr = std::thread([this]()
	{
    auto reconnect_fun_ = [this]()
    {
      reverse_connected_ = false;
      close(new_sockfd_);
      close(incoming_sockfd_);

      // give some time to "ur_communication" to read the robot state, so that protective/emergency stop flags are properly updated
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      ur_::Timer timer;
      timer.start();
      while (isProtectiveStopped() || isEmergencyStopped())
      {
        //std::cerr << "isProtectiveStopped = " << isProtectiveStopped() << "\n";
        //std::cerr << "isEmergencyStopped = " << isEmergencyStopped() << "\n\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (timer.elapsedMilliSec() > 10000)
        {
          print_error("[UrDriver::readRobotStateThread]: 10 sec passed and the robot is still not enabled... Aborting reconnection attempt...\n");
          return;
        }
      }

      // if (isProtectiveStopped()) std::this_thread::sleep_for(std::chrono::seconds(4));

      incoming_sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);
      com_::setReuseAddr(incoming_sockfd_, true);
      com_::bind(incoming_sockfd_, REVERSE_PORT_, host_ip_);
      com_::listen(incoming_sockfd_, 1);
      com_::setNonBlocking(incoming_sockfd_, true);
      //setUrScriptCmd(program_);
      rt_interface_->addCommandToQueue(ur_driver_program_);
      print_debug(UrDriver_fun_ + "Waiting to accept client...\n");
      new_sockfd_ = com_::acceptClient(incoming_sockfd_, com_::Timeout(10,0));
      print_debug(UrDriver_fun_ + "Listening on " + com_::getLocalIp(new_sockfd_) + ":" + std::to_string(com_::getLocalPort(new_sockfd_)) + "\n");
      reverse_connected_ = true;
    };

    std::string fun_name_ = "[UrDriver::readRobotStateThread]: ";

		char buf[4096];

		com_::Timeout tm(0,100000);
		int err_code;
		int bytes_read;

    #ifdef LOG_TIMES
  		ur_::Timer timer;
  		arma::rowvec times_vec(TIME_BUFF_SIZE);
  		int k = 0;
    #endif

    PRINT_INFO_MSG("keep_alive_ = " + std::to_string(keep_alive_) + "\n");

		while (keep_alive_ && reverse_connected_)
		{
			timer.start();

			bytes_read = com_::read(new_sockfd_, buf, 4096, tm, &err_code);

			if (bytes_read > 0)
			{
				com_::setQuickAck(new_sockfd_, true);
				// std::cerr << "Msg: " << std::string(buf, bytes_read) << "\n";
				unpackState(buf, bytes_read);
        update_sem.notify();
			}
      else if (bytes_read == 0)
      {
        print_warning(fun_name_ + "received 0 bytes...\n");
        reverse_connected_ = false;

        update_sem.notify();

        reconnect_fun_();
      }
			else
			{
				if (err_code == EINTR) // ignore "interrupt system call"
				{
					print_warning(UrDriver_fun_ + com_::getErrMsg(EINTR));
					continue;
				}
				print_error(fun_name_ + "Error on \"read()\": " + com_::getErrMsg(err_code));
				// print_info("Realtime port: Is connection lost? Will try to reconnect...\n");
				reverse_connected_ = false;
        update_sem.notify(); // notify in all cases to avoid dead locks...

        if (keep_alive_) reconnect_fun_();
			}


      #ifdef LOG_TIMES
  			times_vec(k++) = timer.elapsedMicroSec();
      #endif
		}

    if (reverse_connected_)
  	{
      terminate();
  		//rt_interface_->addCommandToQueue("stopj(10)\n");
  		reverse_connected_ = false;
  		close(new_sockfd_);
      close(incoming_sockfd_);
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
  	}

    #ifdef LOG_TIMES
  		times_vec = times_vec.subvec(6,k-7); // discard some initial/final values which are noisy
  		double mean_elaps_t = arma::mean(times_vec)/1000;
  		double std_elaps_t = arma::stddev(times_vec, 1)/1000;

  		std::cerr << "==================================\n";
  		std::cerr << "readRobotStateThread:\n";
  		std::cerr << "loop cycle: " << mean_elaps_t << " +/- " << std_elaps_t << " ms\n";
  		std::cerr << "==================================\n";

  		using namespace as64_::io_;
  		FileIO fid("loop_times.bin", FileIO::out | FileIO::trunc);
  		fid.write("elaps_times", times_vec);
  		fid.close();
    #endif

	});
	ur_::makeThreadRT(read_state_thr);
}

void UrDriver::unpackState(char *buff, int size)
{

	int i1 = 0;
	int i2 = 0;

	unpackVector(buff, size, i1, i2, joint_pos);
	unpackVector(buff, size, i1, i2, joint_vel);

	std::vector<double> tcp_pose(6);
	unpackVector(buff, size, i1, i2, tcp_pose);
	unpackVector(buff, size, i1, i2, tcp_vel);
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

	unpackVector(buff, size, i1, i2, joint_torq);
	unpackVector(buff, size, i1, i2, tcp_wrench);
}

void UrDriver::unpackVector(char *buff, int size, int &i1, int &i2, std::vector<double> &unpack_to)
{
	for (int i=i2; i<size; i++)
	{
		if (buff[i] == '[')
		{
			i1 = i;
			break;
		}
		if (buff[i] == ',') buff[i] = ' ';
	}

	for (int i=i1; i<size; i++)
	{
		if (buff[i] == ']')
		{
			i2 = i;
			break;
		}
		if (buff[i] == ',') buff[i] = ' ';
	}

	int i0 = i1+1;
	int len = i2 - i0;
	std::istringstream iss(std::string(buff+i0,len));
	unpack_to.resize(6);
	iss >> unpack_to[0] >> unpack_to[1] >> unpack_to[2] >> unpack_to[3] >> unpack_to[4] >> unpack_to[5];
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
  if (USE_readRTMsg)
  {
    rt_read_thread_ = std::thread(std::bind(&UrDriver::readRTMsg, this));
    int err_code = ur_::makeThreadRT(rt_read_thread_);
    if (err_code) ur_::PRINT_WARNING_MSG("[UrDriver::start]: Failed to set thread priority! Reason:\n" + ur_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  }
  // mb_read_thread_ = std::thread(std::bind(&UrDriver::readMbMsg, this));

  // else PRINT_INFO_MSG("[UrDriver::start]: Set thread priority successfully!\n", std::cerr);

	startReverseCom();

  return true;
}

void UrDriver::halt()
{
  stopReverseCom();
	sec_interface_->halt();
	rt_interface_->halt();
  if (rt_read_thread_.joinable()) rt_read_thread_.join();
}

void UrDriver::readRTMsg()
{
  #ifdef LOG_TIMES
  	ur_::Timer timer;
  	arma::rowvec times_vec(TIME_BUFF_SIZE);
  	int k = 0;
  #endif

  while (keep_alive_)
  {

		timer.start();

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

    #ifdef LOG_TIMES
		  times_vec(k++) = timer.elapsedMicroSec();
    #endif

    if (!ur_script_cmd().empty())
    {
      rt_interface_->addCommandToQueue(ur_script_cmd.get());
      ur_script_cmd.set(""); // clear previous command
    }

    update_sem.notify();
  }

  #ifdef LOG_TIMES
  	times_vec = times_vec.subvec(6,k-7); // discard some initial/final values which are noisy
  	double mean_elaps_t = arma::mean(times_vec)/1000;
  	double std_elaps_t = arma::stddev(times_vec, 1)/1000;

  	std::cerr << "==================================\n";
  	std::cerr << "readRobotStateThread:\n";
  	std::cerr << "loop cycle: " << mean_elaps_t << " +/- " << std_elaps_t << " ms\n";
  	std::cerr << "==================================\n";

  	using namespace as64_::io_;
  	FileIO fid("loop_times2.bin", FileIO::out | FileIO::trunc);
  	fid.write("elaps_times", times_vec);
  	fid.close();
  #endif

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

void UrDriver::idle_mode()
{
	const int len = 4;
	char buff[len];
	writeInt(buff, IDLE_MODE);

	int n_bytes = ur_::com_::write(new_sockfd_, buff, len, true);
	if (n_bytes != len) throw std::runtime_error("Error: sent bytes=" + std::to_string(n_bytes) + ", bytes to send=" + std::to_string(len) + "\n");
}

void UrDriver::terminate()
{
	const int len = 4;
	char buff[len];
	writeInt(buff, TERMINATE);

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
