/*
 * ur_communication.cpp
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

#include <ur_modern_driver/ur_communication.h>

#define UrCom_fun_ std::string("[UrCommunication::") + __func__ + "]: "

using namespace ur_;

UrCommunication::UrCommunication(ur_::Semaphore &msg_sem, std::string host)
{
  this->msg_sem_ptr = &msg_sem;

	connected_ = false;
	keepalive_ = false;

  host_ = host;
  sec_port_ = 30002;
  pri_port_ = 30001;
}

UrCommunication::~UrCommunication()
{
  halt();
}

void UrCommunication::initPri()
{
  pri_sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);
  com_::setNoDelay(pri_sockfd_, true);
  com_::setQuickAck(pri_sockfd_, true);
  com_::setReuseAddr(pri_sockfd_, true);

	print_debug("Acquire firmware version: Connecting...");
  com_::connectToServer(pri_sockfd_, host_, pri_port_);
	print_debug("Acquire firmware version: Got connection");

  uint8_t buf[512];
	unsigned int bytes_read;
	std::string cmd;
	bzero(buf, 512);
	bytes_read = read(pri_sockfd_, buf, 512);
  com_::setQuickAck(pri_sockfd_, true);
	robot_state_.unpack(buf, bytes_read);
	// msg_sem_ptr->notify();
	//wait for some traffic so the UR socket doesn't die in version 3.1.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	char tmp[64];
	sprintf(tmp, "Firmware version detected: %.7f", robot_state_.getVersion());
	print_debug(tmp);
  com_::closeSocket(pri_sockfd_);
}

void UrCommunication::initSec()
{
  sec_sockfd_ = com_::openSocket(AF_INET, SOCK_STREAM);

  com_::setNoDelay(sec_sockfd_, true);
  com_::setQuickAck(sec_sockfd_, true);
  com_::setReuseAddr(sec_sockfd_, true);
  com_::setNonBlocking(sec_sockfd_, true);

  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  com_::connectToServer(sec_sockfd_, host_, sec_port_, timeout);
  connected_ = true;
}

bool UrCommunication::start()
{
	keepalive_ = true;

  print_debug("Connecting to primary interface...");
  initPri();

	print_debug("Switching to secondary interface for masterboard data: Connecting...");
  initSec();
	print_debug("Secondary interface: Got connection");

	comThread_ = std::thread(&UrCommunication::run, this);
	return true;
}

void UrCommunication::halt()
{
	keepalive_ = false;
  if (comThread_.joinable()) comThread_.join();
  msg_sem_ptr->notify();
}

void UrCommunication::run()
{
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);

  struct timeval timeout;
  timeout.tv_sec = 0; //do this each loop as selects modifies timeout
  timeout.tv_usec = 500000; // timeout of 0.5 sec

  int err_code;

  print_debug("Secondary port: Got connection");

	while (keepalive_)
	{
		while (connected_ && keepalive_)
		{
      int err_code;
      com_::WaitResult result = com_::waitForRead(sec_sockfd_, timeout, &err_code);

      if (result == com_::READY)
      {
        bytes_read = ::read(sec_sockfd_, buf, 2048);
  			if (bytes_read > 0)
  			{
          com_::setQuickAck(sec_sockfd_, true);
  				robot_state_.unpack(buf, bytes_read);
  				msg_sem_ptr->notify();
  			}
  			else
        {
          connected_ = false;
  				robot_state_.setDisconnected();
          com_::closeSocket(sec_sockfd_);
  			}
      }
      else // (result != com_::READY)
      {
        if (err_code == EINTR)
        {
          print_warning(UrCom_fun_ + "Interrupt by system call...\n");
          continue;
        }
        if (result == com_::TIMEOUT) print_error(UrCom_fun_ + "Timeout on waitForRead...\n");
        if (result == com_::ERROR) print_error(UrCom_fun_ + "Error on waitForRead: " + com_::getErrMsg(err_code));
        print_info("Realtime port: Is connection lost? Will try to reconnect...\n");
        connected_ = false;
      }
		}

    // attempt reconnect
		if (keepalive_)
		{
      print_warning("Secondary port: Attemting reconnect...");
      com_::closeSocket(sec_sockfd_);
      initSec();
      print_info("Secondary port: Reconnected");
		}
	}

	//wait for some traffic so the UR socket doesn't die in version 3.1.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	com_::closeSocket(sec_sockfd_);
}
