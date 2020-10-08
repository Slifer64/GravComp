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

#include <ur_modern_driver/ur_realtime_communication.h>

#include <ur_modern_driver/utils.h>
#include <ur_modern_driver/socket_com.h>

#include <string>
#include <exception>

#define UrRtCom_fun_ std::string("[UrRealtimeCommunication::") + __func__ + "]: "

using namespace ur_;

UrRealtimeCommunication::UrRealtimeCommunication(ur_::Semaphore &msg_sem, std::string host)
{
  this->msg_sem_ptr = &msg_sem;

  host_ = host;
  port_ = 30003;

	connected_ = false;
	keepalive_ = false;
}

UrRealtimeCommunication::~UrRealtimeCommunication()
{
  halt();
}

void UrRealtimeCommunication::init()
{
  sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);

  com_::setNoDelay(sockfd_, true);
  com_::setQuickAck(sockfd_, true);
  com_::setReuseAddr(sockfd_, true);
  com_::setNonBlocking(sockfd_, true);

  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  com_::connectToServer(sockfd_, host_, port_, timeout);
  connected_ = true;
}

bool UrRealtimeCommunication::start()
{
	keepalive_ = true;
	print_debug("Realtime port: Connecting...");

  init();

	local_ip_ = com_::getLocalIp(sockfd_);

	comThread_ = std::thread(&UrRealtimeCommunication::run, this);
  int err_code = ur_::makeThreadRT(comThread_);
  if (err_code) ur_::PRINT_WARNING_MSG("[UrRealtimeCommunication::start]: Failed to set thread priority! Reason:\n" + ur_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
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
	if (connected_) bytes_written = ::write(sockfd_, inp.c_str(), inp.length());
	else print_error("Could not send command \"" +inp + "\". The robot is not connected! Command is discarded" );
}

void UrRealtimeCommunication::run()
{
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);

  struct timeval timeout;
  timeout.tv_sec = 0; //do this each loop as selects modifies timeout
  timeout.tv_usec = 500000; // timeout of 0.5 sec
  //timeout.tv_usec = 4000; // timeout of 4 ms

  int err_code;

  // wait a bit at first...
  com_::WaitResult result = com_::waitForRead(sockfd_, timeout, &err_code);

  // change timeout
  // timeout.tv_usec = 8000; // timeout of 8 ms

	print_debug("Realtime port: Got connection");

	while (keepalive_)
	{
		while (connected_ && keepalive_)
		{
      com_::WaitResult result = com_::waitForRead(sockfd_, timeout, &err_code);

      if (result == com_::READY)
      {
        bytes_read = ::read(sockfd_, buf, 2048);
  			if (bytes_read > 0)
  			{
          com_::setQuickAck(sockfd_, true);
  				robot_state_.unpack(buf);
  				msg_sem_ptr->notify();
  			}
  			else connected_ = false;
      }
      else // (result != com_::READY)
      {
        if (err_code == EINTR)
        {
          print_warning(UrRtCom_fun_ + "Interrupt by system call...\n");
          continue;
        }
        if (result == com_::TIMEOUT) print_error(UrRtCom_fun_ + "Timeout on waitForRead...\n");
        if (result == com_::ERROR) print_error(UrRtCom_fun_ + "Error on waitForRead: " + com_::getErrMsg(err_code));
        print_info("Realtime port: Is connection lost? Will try to reconnect...\n");
        connected_ = false;
      }

		}

    // attempt reconnect
		if (keepalive_)
		{
			print_warning("Realtime port: Attemting reconnect...");
      com_::closeSocket(sockfd_);
      init();
      print_info("Realtime port: Reconnected");
		}
	}

  com_::closeSocket(sockfd_);
}

std::string UrRealtimeCommunication::getLocalIp()
{
	return local_ip_;
}


// ===================================================================
// ===================================================================
