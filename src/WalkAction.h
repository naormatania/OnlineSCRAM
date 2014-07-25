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
#include <sensor_msgs/LaserScan.h>

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

bool obstacleFound = false;

void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr &sensor_msg);

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
  ros::Subscriber baseScanSubscriber_;

public:

  WalkAction(std::string name, int robot_id) :
    as_(nh_, name, false),
    action_name_(name), robot_id_(robot_id)
  {

	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&WalkAction::goalCB, this));
	// as_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

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

	// subscribe to robot's laser scan topic "base_scan"
	string laser_scan_topic_name = "base_scan";

	baseScanSubscriber_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_scan_topic_name, 1,
            &WalkAction::readSensorCallback);

    as_.start();
  }

  ~WalkAction(void)
  {
	  delete moveBaseClient_;
  }

  void goalCB()
  {
	// online_scram::WalkGoalConstPtr
	online_scram::WalkGoalConstPtr goal = as_.acceptNewGoal();
	// helper variables
    bool success = true;
    string tf_prefix;
    nh_.getParam("tf_prefix", tf_prefix);

    // push_back the seeds for the fibonacci sequence
    feedback_.is_waiting = false;
    feedback_.robot_id = robot_id_;

  	tf::TransformListener listener;
  	string this_robot_frame = tf::resolve(tf_prefix, "base_footprint");

  	// wait to have some transform before start
  	listener.waitForTransform("/map", this_robot_frame, ros::Time(0), ros::Duration(10.0));

  	ros::Rate fasterLoopRate(20);
  	while (ros::ok()) {
  		as_.publishFeedback(feedback_);

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

  		double x_diff = goal->x - transform.getOrigin().x();
  		double y_diff = goal->y - transform.getOrigin().y();
  		double dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
  		double theta =  atan2(y_diff,x_diff) - tf::getYaw(transform.getRotation());
  		x_diff = dist*cos(theta);
  		y_diff = dist*sin(theta);

		geometry_msgs::Twist vel_msg;

		if (dist > NEGLIBLE_DISTANCE) {
			if ((obstacleFound == true) && (goal->c == true)) {
				ROS_WARN("ROBOT %d is waiting", robot_id_);
				vel_msg.angular.z = M_PI/2;
				feedback_.is_waiting = true;
			}
			else {
				vel_msg.linear.x = min(dist, MAX_LINEAR_VEL);
				vel_msg.angular.z = min(4 * atan2(y_diff,x_diff), MAX_ANGULAR_VEL);
				feedback_.is_waiting = false;
			}
		}
		else {
			feedback_.is_waiting = false;
			as_.setSucceeded(result_);
			break;
		}
		cmdVelPublisher_.publish(vel_msg);

  		ros::spinOnce();
  		fasterLoopRate.sleep();
  	}
  }

  static void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr &sensor_msg)
  {
      int arraySize = (sensor_msg->angle_max - sensor_msg->angle_min) /
              sensor_msg->angle_increment;

      bool isObstacle = false;

      for (int i = 0; i < arraySize; i++) {
          if ((sensor_msg->ranges[i] > sensor_msg->range_min) &&
        		  (sensor_msg->ranges[i] < sensor_msg->range_max) &&
        		  (sensor_msg->ranges[i] < 0.5)) {
              isObstacle = true;
          }
      }


      if (isObstacle)
      {
          obstacleFound = true;
      } else {
          obstacleFound = false;
      }
  }

};
#endif /* WALKACTION_CPP_ */
