/*
 * ur_realtime_communication.cpp
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

#include "ur_modern_driver/ur_realtime_communication.h"

#include <ur_modern_driver/utils.h>

UrRealtimeCommunication::UrRealtimeCommunication(as64_::ur_::Semaphore &msg_sem, std::string host, unsigned int safety_count_max)
{
  this->msg_sem_ptr = &msg_sem;
	bzero((char *) &serv_addr_, sizeof(serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) print_fatal("ERROR opening socket");
	server_ = gethostbyname(host.c_str());
	if (server_ == NULL) print_fatal("ERROR, no such host");
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);
	// inet_pton(AF_INET, host_ip.c_str(), (void *)&server_addr.sin_addr.s_addr);
	serv_addr_.sin_port = htons(30003);
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
	fcntl(sockfd_, F_SETFL, O_NONBLOCK); // make the socket non-blocking
	connected_ = false;
	keepalive_ = false;
	safety_count_ = safety_count_max + 1;
	safety_count_max_ = safety_count_max;
}

UrRealtimeCommunication::~UrRealtimeCommunication()
{
  halt();

//  std::cerr << "[UrRealtimeCommunication::~UrRealtimeCommunication]: DONE!\n";
}

bool UrRealtimeCommunication::start()
{
	fd_set writefds;
	struct timeval timeout;

	keepalive_ = true;
	print_debug("Realtime port: Connecting...");

	connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
	FD_ZERO(&writefds);
	FD_SET(sockfd_, &writefds);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	select(sockfd_ + 1, NULL, &writefds, NULL, &timeout);
	unsigned int flag_len;
	getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
	if (flag_ < 0)
	{
		print_fatal("Error connecting to RT port 30003");
		return false;
	}
	sockaddr_in name;
	socklen_t namelen = sizeof(name);
	int err = getsockname(sockfd_, (sockaddr*) &name, &namelen);
	if (err < 0)
	{
		print_fatal("Could not get local IP");
		close(sockfd_);
		return false;
	}
	char str[18];
	inet_ntop(AF_INET, &name.sin_addr, str, 18);
	local_ip_ = str;
	comThread_ = std::thread(&UrRealtimeCommunication::run, this);
  int err_code = as64_::ur_::makeThreadRT(comThread_);
  if (err_code) as64_::ur_::PRINT_WARNING_MSG("[UrRealtimeCommunication::start]: Failed to set thread priority! Reason:\n" + as64_::ur_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG("[UrDriver::start]: Set thread priority successfully!\n", std::cerr);

  return true;
}

void UrRealtimeCommunication::halt()
{
	keepalive_ = false; // it is important to do this before the final notification to avoid deadlocks!
	if (comThread_.joinable()) comThread_.join();
  this->msg_sem_ptr->notify();
}

void UrRealtimeCommunication::addCommandToQueue(std::string inp)
{
	int bytes_written;
	if (inp.back() != '\n') inp.append("\n");
	if (connected_) bytes_written = write(sockfd_, inp.c_str(), inp.length());
	else print_error("Could not send command \"" +inp + "\". The robot is not connected! Command is discarded" );
}

void UrRealtimeCommunication::setSpeed(double q0, double q1, double q2, double q3, double q4, double q5, double acc)
{
	char cmd[1024];
	if( robot_state_.getVersion() >= 3.3 ) sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.008)\n", q0, q1, q2, q3, q4, q5, acc);
	else if( robot_state_.getVersion() >= 3.1 ) sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f)\n", q0, q1, q2, q3, q4, q5, acc);
	else sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n", q0, q1, q2, q3, q4, q5, acc);
	addCommandToQueue((std::string) (cmd));
	if (q0 != 0. or q1 != 0. or q2 != 0. or q3 != 0. or q4 != 0. or q5 != 0.)
	{
		//If a joint speed is set, make sure we stop it again after some time if the user doesn't
		safety_count_ = 0;
	}
}

void UrRealtimeCommunication::run()
{
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);
	struct timeval timeout;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd_, &readfds);
	print_debug("Realtime port: Got connection");
	connected_ = true;

	//ur_::Timer timer;
	//std::vector<double> times_vec;

	while (keepalive_)
	{
		while (connected_ && keepalive_)
		{
      //timer.start();

			timeout.tv_sec = 0; //do this each loop as selects modifies timeout
			timeout.tv_usec = 500000; // timeout of 0.5 sec
			select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
			bytes_read = read(sockfd_, buf, 2048);
			if (bytes_read > 0)
			{
				setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, sizeof(int));
				robot_state_.unpack(buf);
				msg_sem_ptr->notify();
				// if (safety_count_ == safety_count_max_) setSpeed(0., 0., 0., 0., 0., 0.);
				// safety_count_ += 1;
			}
			else
      {
				connected_ = false;
				close(sockfd_);
			}

			//double elaps_time = timer.elapsedMicroSec();
			//times_vec.push_back(elaps_time);
		}

		if (keepalive_)
		{
			//reconnect
			print_warning("Realtime port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
			sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd_ < 0) print_fatal("ERROR opening socket");
			flag_ = 1;
			setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
			setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, sizeof(int));
			setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
			fcntl(sockfd_, F_SETFL, O_NONBLOCK);
			while (keepalive_ && !connected_)
			{
				std::this_thread::sleep_for(std::chrono::seconds(10));
				fd_set writefds;
				connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));
				FD_ZERO(&writefds);
				FD_SET(sockfd_, &writefds);
				select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
				unsigned int flag_len;
				getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
				if (flag_ < 0)
				{
					print_error("Error re-connecting to RT port 30003. Is controller started? Will try to reconnect in 10 seconds...");
				}
				else
        {
					connected_ = true;
					print_info("Realtime port: Reconnected");
				}
			}
		}
	}

//	int N_times = times_vec.size();
//	double mean_time=0, max_time = 0, min_time = 1e12;
//  for (int i=0;i<N_times; i++)
//  {
//    double elaps_time = times_vec[i];
//    mean_time += elaps_time;
//    if (elaps_time > max_time) max_time = elaps_time;
//    if (elaps_time < min_time) min_time = elaps_time;
//  }
//  mean_time /= N_times;
//  std::cerr << "======================================\n";
//  std::cerr << "mean_time: " << mean_time/1000 << " ms\n";
//  std::cerr << "min_time: " << min_time/1000 << " ms\n";
//  std::cerr << "max_time: " << max_time/1000 << " ms\n";
//  std::cerr << "======================================\n";

	setSpeed(0., 0., 0., 0., 0., 0.);
	close(sockfd_);
}

void UrRealtimeCommunication::setSafetyCountMax(uint inp)
{
	safety_count_max_ = inp;
}

std::string UrRealtimeCommunication::getLocalIp()
{
	return local_ip_;
}
