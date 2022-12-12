#include <ros/ros.h>
#include "TSPAction.h"

using namespace std;

//+++++++++++++++++++++++++++++++ MAIN ++++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {
	// printData();
	ros::init(argc, argv, "robot_router");
	
	TSPAction tsp("tsp");
	
	ros::spin();
	return 0;
}
