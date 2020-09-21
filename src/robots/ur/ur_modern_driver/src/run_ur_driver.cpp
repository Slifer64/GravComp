#include "ur_modern_driver/ur_ros_wrapper.h"

int main(int argc, char **argv)
{
	bool use_sim_time = false;
	std::string host;
	int reverse_port = 50001;

	ros::init(argc, argv, "ur_driver");
	ros::NodeHandle nh;
	if (ros::param::get("use_sim_time", use_sim_time)) {
		print_warning("use_sim_time is set!!");
	}
	if (!(ros::param::get("~robot_ip_address", host))) {
		if (argc > 1) {
			print_warning(
					"Please set the parameter robot_ip_address instead of giving it as a command line argument. This method is DEPRECATED");
			host = argv[1];
		} else {
			print_fatal(
					"Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
			exit(1);
		}

	}
	if ((ros::param::get("~reverse_port", reverse_port))) {
		if((reverse_port <= 0) or (reverse_port >= 65535)) {
			print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 50001");
			reverse_port = 50001;
		}
	} else
		reverse_port = 50001;

	ur_::RosWrapper interface(host, reverse_port);

	ros::waitForShutdown();

	exit(0);
}
