/*
 * controller.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: naor
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "WalkAction.h"

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "walker");
	ros::NodeHandle nh;
	string tf_prefix;

	nh.getParam("tf_prefix", tf_prefix);
	int robot_id = int(tf_prefix[6])-int('0');
	ROS_INFO("Robot id is %d", robot_id);

	WalkAction walk(ros::this_node::getName(), robot_id);
	ros::spin();

	ROS_INFO("Ready to start walking.");
	ros::spin();
	return 0;
}
