/*
 * WalkAction.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: naor
 */

#ifndef WALKACTION_CPP_
#define WALKACTION_CPP_

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Twist.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <online_scram/WalkAction.h>

#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
using namespace boost;

#define MAX_LINEAR_VEL 0.7
#define MAX_ANGULAR_VEL 3.14
#define NEGLIBLE_DISTANCE 0.05 //5 centimeters

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WalkAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<online_scram::WalkAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  online_scram::WalkFeedback feedback_;
  online_scram::WalkResult result_;
  int robot_id_;
  MoveBaseClient *moveBaseClient_;

  ros::Publisher cmdVelPublisher_;

public:

  WalkAction(std::string name, int robot_id) :
    as_(nh_, name, boost::bind(&WalkAction::executeCB, this, _1), false),
    action_name_(name), robot_id_(robot_id)
  {
	string cmd_vel_str = "/robot_";
	cmd_vel_str += boost::lexical_cast<string>(robot_id);
	cmd_vel_str += "/cmd_vel";
	cmdVelPublisher_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_str, 10);

	string move_base_str = "/robot_";
	move_base_str += boost::lexical_cast<string>(robot_id);
	move_base_str += "/move_base";
	ROS_INFO("move_base_str is %s",move_base_str.c_str());
	moveBaseClient_ = new MoveBaseClient(move_base_str, true);
	// Wait for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	moveBaseClient_->waitForServer(ros::Duration(5));

    as_.start();
  }

  ~WalkAction(void)
  {
	  delete moveBaseClient_;
  }

  void executeCB(const online_scram::WalkGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    string tf_prefix;
    nh_.getParam("tf_prefix", tf_prefix);

    // push_back the seeds for the fibonacci sequence
    feedback_.is_waiting = false;

  	tf::TransformListener listener;
  	string this_robot_frame = tf::resolve(tf_prefix, "base_footprint");

  	// wait to have some transform before start
  	listener.waitForTransform("/map", this_robot_frame, ros::Time(0), ros::Duration(10.0));

  	tf::StampedTransform transform;
  	try {
		listener.lookupTransform("/map", this_robot_frame, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}

	//double theta = atan2(transform.getOrigin().y() - goal->y,
	//  				transform.getOrigin().x() - goal->x);
	double theta = M_PI/2;

	move_base_msgs::MoveBaseGoal moveBaseGoal;
	moveBaseGoal.target_pose.header.frame_id = "map";
	moveBaseGoal.target_pose.header.stamp = ros::Time::now();

	moveBaseGoal.target_pose.pose.position.x = 10;
	moveBaseGoal.target_pose.pose.position.y = 10;

	// Convert the Euler angle to quaternion
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(theta);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);
	moveBaseGoal.target_pose.pose.orientation = qMsg;

	ros::Rate loopRate(10);
	ROS_INFO("Sending goal to change angle with x=%f,y=%f", transform.getOrigin().x(),
			transform.getOrigin().y());
	moveBaseClient_->sendGoal(moveBaseGoal);
	ROS_INFO("Waiting for result");
	ros::Duration(1).sleep();
	ROS_INFO("state is %s", moveBaseClient_->getState().toString().c_str());
	moveBaseClient_->waitForResult();
	ROS_INFO("result reached");

  	ros::Rate fasterLoopRate(10);
  	while (ros::ok()) {

        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
          break;
        }

  		tf::StampedTransform transform;

  		try {
  			listener.lookupTransform("/map", this_robot_frame, ros::Time(0), transform);
  		}
  		catch (tf::TransformException &ex) {
  			ROS_ERROR("%s",ex.what());
  		}

  		float dist = sqrt(pow(transform.getOrigin().x() - goal->x, 2) +
                  pow(transform.getOrigin().y() - goal->y, 2));

  		if (dist < NEGLIBLE_DISTANCE) {
  			result_.total_dishes_cleaned = 3;
  			ROS_INFO("%s: Succeeded", action_name_.c_str());
  			// set the action state to succeeded
  			as_.setSucceeded(result_);
  			success = true;
  			break;
  		}

  		geometry_msgs::Twist vel_msg;

		vel_msg.linear.x = min(dist, float(MAX_LINEAR_VEL));
		vel_msg.angular.z = 0;

  		cmdVelPublisher_.publish(vel_msg);

  		ros::spinOnce();
  		fasterLoopRate.sleep();
  	}
  };

};

#endif /* WALKACTION_CPP_ */
