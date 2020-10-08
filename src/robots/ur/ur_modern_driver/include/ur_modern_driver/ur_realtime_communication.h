/*
 * ur_realtime_communication.h
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

#ifndef UR_REALTIME_COMMUNICATION_H_
#define UR_REALTIME_COMMUNICATION_H_

#include <ur_modern_driver/robot_state_RT.h>
#include <ur_modern_driver/do_output.h>
#include <ur_modern_driver/utils.h>

class UrRealtimeCommunication
{
public:

	bool connected_;
	RobotStateRT robot_state_;

	UrRealtimeCommunication(ur_::Semaphore &msg_sem, std::string host);
	~UrRealtimeCommunication();
	bool start();
	void halt();

	void addCommandToQueue(std::string inp);
	std::string getLocalIp();

private:
	int sockfd_;
	bool keepalive_;
	std::thread comThread_;

  ur_::Semaphore *msg_sem_ptr;

	std::string local_ip_;
	std::string host_;
	int port_;

	void init();
	void run();
};


#endif /* UR_REALTIME_COMMUNICATION_H_ */
