/*
 * assigner.cpp
 *
 *  Created on: June, 2014
 *      Author: naorm
 */

#include <ros/ros.h>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <costmap_2d/costmap_2d_ros.h>

#include "geometry_msgs/Pose2D.h"

#include "straight_planner.h"

#include "common.h"
#include "mmdr.h"
#include "mmd_msd2.h"

#include <online_scram/WalkAction.h>
#include <online_scram/WalkActionFeedback.h>
#include <online_scram/WalkActionResult.h>
#include <online_scram/WalkGoal.h>
#include <online_scram/WalkFeedback.h>
#include <online_scram/WalkActionGoal.h>
#include <online_scram/WalkResult.h>

using namespace std;
using namespace boost;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<online_scram::WalkAction> WalkClient;

#define NUM_ROBOTS 5
#define NUM_RUNS 20
#define GRID_WIDTH 15
#define LOGGER_NAME "ScramOnline.log"

void moveRobot(WalkClient *ac, std::pair<int,int> location, bool care_collision = true);

bool sortEdges(const Edge &e1,const Edge &e2) { return (e1.first<e2.first); }

typedef std::vector<Edge> AssignmentAlgo(Test t);

std::auto_ptr<std::pair<Test,std::vector<Edge> > > lastAssignment(NULL);

bool is_waiting[NUM_ROBOTS] = {false, false, false, false, false};

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const online_scram::WalkResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const online_scram::WalkFeedbackConstPtr& feedback)
{
  if (feedback->is_waiting == true) {
	  ROS_WARN("Robot id %d with feed_back %d", feedback->robot_id,
	  			feedback->is_waiting);
  }
  is_waiting[feedback->robot_id] = feedback->is_waiting;
}

std::vector<Edge> BLE(Test t){
	std::vector<Edge> all_distances;
	std::vector<Edge> answer;

	int n = t.starts.size();
	int m = t.targets.size();

	bool is_robot_taken[N];
	bool is_target_taken[N];

	for (int i=0; i < N; i++) {
		is_robot_taken[i] = false;
		is_target_taken[i] = false;
	}

	for (int i=0; i < N; i++) {
		for (int j=0; j < N; j++) {
			double dist = 1000;
			if ((i < n) && (j < m)) {
				dist = getdist(t.starts[i], t.targets[j]);
			}
			all_distances.push_back(Edge(dist,std::make_pair(i,j)));
		}
	}
	std::sort(all_distances.begin(), all_distances.end(), sortEdges);
	for (std::vector<Edge>::iterator it=all_distances.begin(); it!=all_distances.end(); ++it) {
		if ((is_robot_taken[it->second.first] == false) && (is_target_taken[it->second.second] == false)) {
			answer.push_back(*it);
			is_robot_taken[it->second.first] = true;
			is_target_taken[it->second.second] = true;
		}
		if (answer.size() == std::min(m,n)) {
			break;
		}
	}

	return answer;
}

std::vector<Edge> MURDOCH(Test t){
	int n = t.starts.size();
	int m = t.targets.size();

	std::vector<Edge> all_distances;
	std::vector<Edge> answer;
	if (lastAssignment.get() == NULL) {
		answer = BLE(t);
	}
	else {
		Test lastTest = lastAssignment.get()->first;
		std::vector<Edge> lastAnswer = lastAssignment.get()->second;
		std::vector<Point> diff;
		// we assume here the next target won't be on the same location as the previous one
		std::set_difference(lastTest.targets.begin(), lastTest.targets.end(),
				t.targets.begin(), t.targets.end(),
		                        std::inserter(diff, diff.begin()));

		int lastFinishedTarget = -1;
		int lastFinishedRobot = -1;
		for (int i=0; i < lastTest.targets.size(); i++) {
			if (lastTest.targets[i] == diff[0]) {
				lastFinishedTarget = i;
				break;
			}
		}

		for (int i=0; i < lastAnswer.size(); i++) {
			if (lastFinishedTarget == lastAnswer[i].second.second) {
				lastFinishedRobot = lastAnswer[i].second.first;
				break;
			}
		}

		for (int i=0; i < lastAnswer.size(); i++) {
			int robotIndex = lastAnswer[i].second.first;
			int targetIndex = lastAnswer[i].second.second;
			if (targetIndex == lastFinishedTarget) {
				continue;
			}
			if (targetIndex > lastFinishedTarget) {
				--targetIndex;
			}
			double dist = getdist(t.starts[robotIndex], t.targets[targetIndex]);
			answer.push_back(make_pair(dist, make_pair(robotIndex,targetIndex)));
		}
		answer.push_back(
			make_pair(getdist(t.starts[lastFinishedRobot], t.targets[m-1]),
				make_pair(lastFinishedRobot,m-1)));
	}

	lastAssignment.reset(new pair<Test,vector<Edge> >(t,answer));
	return answer;
}

std::pair<std::pair<int,int>,double> getRobotLocation(int robot_index) {
	tf::TransformListener listener;

	string map_frame = "/map";
	string robot_str = "/robot_";
	robot_str += boost::lexical_cast<string>(robot_index);
	string robot_frame = tf::resolve(robot_str, "base_footprint");

	listener.waitForTransform(map_frame, robot_frame, ros::Time(0), ros::Duration(10.0));
	tf::StampedTransform transform;
	try {
		listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}

	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double theta = tf::getYaw(transform.getRotation());

	return std::make_pair(
			std::make_pair(round(x), round(y)),theta);
}

inline double getAngleRobotToTarget(Point &a, Point &b) {
	return atan2(b.second-a.second, b.first-a.first);
}

void adjustAngle(ros::Publisher &cmd_vel_pub, double curAngle, double newAngle) {
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = newAngle - curAngle;
	cmd_vel_pub.publish(vel_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "assigner");
	ros::NodeHandle nh;
	srand (time(NULL));

	std::vector<std::pair<std::string,void*> > AssignmentAlgorithms;

	AssignmentAlgorithms.push_back(std::make_pair(string("BLE"), (void*)BLE));
	AssignmentAlgorithms.push_back(std::make_pair(string("MURDOCH"), (void*)MURDOCH));
	AssignmentAlgorithms.push_back(std::make_pair(string("mmdr"), (void*)mmdr_n5));
	AssignmentAlgorithms.push_back(std::make_pair(string("mmd_msd2"), (void*)mmd_msd2));

	std::vector<MoveBaseClient*> MoveBaseClientList;
	std::vector<WalkClient*> WalkClientList;
	std::vector<ros::Publisher> CmdVelPublishersList;
	std::vector<ros::Publisher> PosePublishersList;

	for (int robot_id = 0; robot_id < NUM_ROBOTS; ++robot_id) {
		ROS_INFO("Trying to connect to move base robot %d", robot_id);
		/*
		// Create the string "robot_X/move_base"
		string move_base_str = "/robot_";
		move_base_str += boost::lexical_cast<string>(robot_id);
		move_base_str += "/move_base";

		// create the action client
		MoveBaseClient *ac = new MoveBaseClient(move_base_str, true);

		// Wait for the action server to become available
		ROS_INFO("Waiting for the move_base action server");
		ac->waitForServer(ros::Duration(5));

		ROS_INFO("Connected to move base server");
		MoveBaseClientList.push_back(ac);
		*/
		// Create the string "robot_X/move_base"
		string move_base_str = "/robot_";
		move_base_str += boost::lexical_cast<string>(robot_id);
		move_base_str += "/walker";

		// create the action client
		WalkClient *ac = new WalkClient(move_base_str, true);

		// Wait for the action server to become available
		ROS_INFO("Waiting for the move_base action server");
		ac->waitForServer(ros::Duration(5));

		ROS_INFO("Connected to move base server");
		WalkClientList.push_back(ac);

		string cmd_vel_str = "/robot_";
		cmd_vel_str += boost::lexical_cast<string>(robot_id);
		cmd_vel_str += "/cmd_vel";
		CmdVelPublishersList.push_back(nh.advertise<geometry_msgs::Twist>(cmd_vel_str, 10));
	}

	for (int robot_id = 0; robot_id < NUM_ROBOTS; ++robot_id) {
		ROS_INFO("Trying to connect to pose robot %d", robot_id);
		// Create the string "robot_X/move_base"
		string pose_str = "/robot_";
		pose_str += boost::lexical_cast<string>(NUM_ROBOTS+robot_id);
		pose_str += "/pose";

		PosePublishersList.push_back(nh.advertise<geometry_msgs::Pose2D>(pose_str, 10));
	}

	for (int run_index = 0; run_index < NUM_RUNS; run_index++) {
		ROS_INFO("Run number %d", run_index);
		string log_name = boost::lexical_cast<string>(run_index)+"_"+LOGGER_NAME;
		std::ofstream log(log_name.c_str(),std::ofstream::binary);
		std::vector<double> robotsAngles;

		ROS_INFO("Generating goals for run");
		std::vector<Point> RunGoalsList;
		for (int i = 0; i < NUM_GOALS; i++) {
			pair<int,int> goal(1+rand()%GRID_WIDTH, 1+rand()%GRID_WIDTH);
			RunGoalsList.push_back(goal);
		}

		log << "Goals:";
		for (int i = 0; i < RunGoalsList.size(); i++) {
			log << " (" << RunGoalsList[i].first << "," << RunGoalsList[i].second << ")";
		}
		log << std::endl;

		for (std::vector<std::pair<std::string,void*> >::iterator it=AssignmentAlgorithms.begin();
				it!=AssignmentAlgorithms.end(); ++it)
		{
			// log assignment algorithm type
			log << "/" << it->first << std::endl;
			std::vector<Point> GoalsList = RunGoalsList;
			time_t start_time = time(NULL);

			while (GoalsList.size() > 0) {
				int n = NUM_ROBOTS;

				Test t;

				// retrieve current robots locations
				for (int i = 0; i < n; i++) {
				  std::pair<std::pair<int,int>,double> pair = getRobotLocation(i);
				  t.starts.push_back(pair.first);
				}

				// retrieve the next goal locations
				for (int i = 0; i < n; i++) {
				  t.targets.push_back(GoalsList.back());
				  GoalsList.pop_back();
				}

				// put the targets on the stage map
				for (int i = 0; i < n; i++) {
					Point p = t.targets[i];
					geometry_msgs::Pose2D msg;
					msg.x = p.first;
					msg.y = p.second;
					msg.theta = 0;
					PosePublishersList[i].publish(msg);
				}

				ROS_INFO("Trying to run task assignment algorithm");
				// solve the task assignment problem
				std::vector<Edge> answer =
						((AssignmentAlgo*)(it->second))(t);
				ROS_INFO("Success !!!");

				// log the task assignment chosen & and the time before performing it
				log << "Time " << time(NULL) - start_time << std::endl;

				log << "Robots Locations";
				for (int i = 0; i < n; i++) {
					log << " " << i << "-(" << t.starts[i].first << "," << t.starts[i].second << ")";
				}
				log << std::endl;

				log << "Target Locations: ";
				for (int i = 0; i < n; i++) {
					log << " " << i << "-(" << t.targets[i].first << "," << t.targets[i].second << ")";
				}
				log << std::endl;

				log << "Task Assignment:";
				for (std::vector<Edge>::iterator it=answer.begin(); it!=answer.end(); ++it) {
					int robot_index = it->second.first;
					pair<double, double> robot_location = t.starts[robot_index];
					int target_index = it->second.second;
					pair<double, double> target_location = t.targets[target_index];

					log << " " << robot_index << "-" << target_index;

					ROS_INFO("Trying to move robot %d", robot_index);
					double angle = getAngleRobotToTarget(robot_location,
							target_location);

					// adjustAngle(CmdVelPublishersList[robot_index], robotsAngles[robot_index], angle);
					// getRobotToLocation(CmdVelPublishersList[robot_index], robot_location, target_location);
					moveRobot(WalkClientList[robot_index],
							t.targets[target_index]);
				}
				log << std::endl;

				bool finished = false;
				int robot_finished = -1;
				geometry_msgs::Twist vel_msg;
				// we won't deal with face to face robots here.
				vel_msg.linear.x = 0;
				vel_msg.angular.z = M_PI/2;
				while (!finished) {
					for (int i=0; i< NUM_ROBOTS; i++) {
						if (WalkClientList[i]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
							ROS_INFO("Robot %d has reached the goal!", i);
							robot_finished = i;
							finished = true;
						}
					}
					/*
					for (int j =0; j < NUM_ROBOTS; j++) {
						if (is_waiting[j] == true) {
							CmdVelPublishersList[j].publish(vel_msg);
							// break;
						}
					}
					*/
				}
				log << "Robot " << robot_finished << " finished At time " << time(NULL) - start_time << std::endl;

				if (GoalsList.size() == 0) {
					int num_robots_left = NUM_ROBOTS - 1;
					bool robots_state[NUM_ROBOTS];
					memset(robots_state,false,NUM_ROBOTS);
					robots_state[robot_finished] = true;
					while (num_robots_left > 0) {
						for (int i=0; i< NUM_ROBOTS; i++) {
							if ((WalkClientList[i]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) \
									&& (robots_state[i] == false)){
								ROS_INFO("Robot %d has reached the goal!", i);
								log << "Robot " << i << " finished At time " << time(NULL) - start_time << std::endl;
								robots_state[i] = true;
								num_robots_left--;
							}
						}
					}

					break;
				}

				for (int i = 0; i < NUM_ROBOTS; i++) {
					WalkClientList[i]->cancelAllGoals();
					/*
					tf::TransformListener tf(ros::Duration(10));
					string robot_str = "/robot_";
					robot_str += boost::lexical_cast<string>(i);
					costmap_2d::Costmap2DROS local_costmap(robot_str+"/move_base_node/local_costmap", tf);
					local_costmap.resetLayers();
					costmap_2d::Costmap2DROS global_costmap(robot_str+"/move_base_node/global_costmap", tf);
					global_costmap.resetLayers();
					*/
				}
				int target_finished = -1;
				for (std::vector<Edge>::iterator it=answer.begin();
					it!=answer.end(); ++it) {
					if (it->second.first == robot_finished) {
						target_finished = it->second.second;
						break;
					}
				}

				for (int i=t.targets.size()-1; i>=0; --i) {
					if (i != target_finished) {
						// return the goal back
						GoalsList.push_back(t.targets[i]);
					}
				}
			}
			// move robots back to their spots (30,30) for another run
			for (int i=0; i< NUM_ROBOTS; i++) {
				moveRobot(WalkClientList[i],
					Point(GRID_WIDTH*(i+1)/5,GRID_WIDTH*(i+1)/5), false);
			}
			for (int i=0; i< NUM_ROBOTS; i++) {
				WalkClientList[i]->waitForResult();
			}
		}
		log.close();
	}

	return 0;
}

void moveRobot(WalkClient *ac, std::pair<int,int> location, bool care_collision)
{
	/*
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = location.first;
	goal.target_pose.pose.position.y = location.second;

	// Convert the Euler angle to quaternion
	double radians = angle * (M_PI/180);
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);
	goal.target_pose.pose.orientation = qMsg;

	ros::Rate loopRate(10);

	ac->sendGoal(goal);

	// ac.waitForResult();

	*/
	online_scram::WalkGoal goal;
	goal.x = location.first;
	goal.y = location.second;
	goal.c = care_collision;
	ros::Rate loopRate(10);

	ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

}
