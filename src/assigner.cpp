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
#define FREQUENCY 5.0
// could be 1 or 2
#define INJECT_METHOD 2

int num_collisions = 0;

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
	  num_collisions = num_collisions + 1;
  }
  is_waiting[feedback->robot_id] = feedback->is_waiting;
}

std::vector<Edge> BLE(Test t){
	std::vector<Edge> all_distances;
	std::vector<Edge> answer;

	int n = t.starts.size();
	int m = t.targets.size();

	bool *is_robot_taken = new bool[n];
	bool *is_target_taken = new bool[m];

	for (int i=0; i < n; i++) {
		is_robot_taken[i] = false;
	}
	for (int i=0; i < m; i++) {
		is_target_taken[i] = false;
	}

	for (int i=0; i < n; i++) {
		for (int j=0; j < m; j++) {
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

	delete []is_robot_taken;
	delete []is_target_taken;

	return answer;
}

int robot_finished = -1;

std::vector<Edge> MURDOCH(Test t){
	int n = t.starts.size();
	int m = t.targets.size();

	std::vector<Edge> all_distances;
	std::vector<Edge> answer;
	if (lastAssignment.get() == NULL) {
		answer = BLE(t);
	}
	else if (robot_finished != -1) {
		Test lastTest = lastAssignment.get()->first;
		std::vector<Edge> lastAnswer = lastAssignment.get()->second;

		int lastFinishedRobot = robot_finished;
		int lastFinishedTarget = -1;
		for (int i=0; i < lastAnswer.size(); i++) {
			if (lastFinishedRobot == lastAnswer[i].second.first) {
				lastFinishedTarget = lastAnswer[i].second.second;
				break;
			}
		}

		vector<int> robotsAssigned;
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
			robotsAssigned.push_back(targetIndex);
		}

		std::vector<Edge> dist_list;
		for (int i=0; i < m; i++) {
			bool f= false;
			for (int j = 0; j <n - 1; j++) {
				if (robotsAssigned[j]==i) {
					f = true;
				}
			}
			if (f) {
				continue;
			}
			double dist = getdist(t.starts[lastFinishedRobot], t.targets[i]);
			dist_list.push_back(Edge(dist,std::make_pair(lastFinishedRobot,i)));
		}
		if (dist_list.size() > 0) {
			std::sort(dist_list.begin(), dist_list.end(), sortEdges);
			answer.push_back(dist_list[0]);
		}


	}
	else {
		answer = lastAssignment.get()->second;
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

int NUM_CUR_GOALS = 0;
bool shouldAbort = false;
#if (INJECT_METHOD == 2)
void timerCallback(const ros::TimerEvent&) {
	ROS_WARN("Timer called");
	NUM_CUR_GOALS++;
	shouldAbort = true;
}
#endif

int main(int argc, char** argv) {
	ros::init(argc, argv, "assigner");
	ros::NodeHandle nh;
	srand (time(NULL));

	std::vector<std::pair<std::string,void*> > AssignmentAlgorithms;

	AssignmentAlgorithms.push_back(std::make_pair(string("MURDOCH"), (void*)MURDOCH));
	AssignmentAlgorithms.push_back(std::make_pair(string("BLE"), (void*)BLE));
	AssignmentAlgorithms.push_back(std::make_pair(string("mmdr"), (void*)mmdr_n5));
	AssignmentAlgorithms.push_back(std::make_pair(string("mmd_msd2"), (void*)mmd_msd2));

	std::vector<MoveBaseClient*> MoveBaseClientList;
	std::vector<WalkClient*> WalkClientList;
	std::vector<ros::Publisher> CmdVelPublishersList;
	std::vector<ros::Publisher> PosePublishersList;

	for (int robot_id = 0; robot_id < NUM_ROBOTS; ++robot_id) {
		ROS_INFO("Trying to connect to move base robot %d", robot_id);

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
			Point goal;
			while (true) {
				goal = std::make_pair(1+rand()%GRID_WIDTH, 1+rand()%GRID_WIDTH);
				bool found = false;
				for (int j = 0; j < RunGoalsList.size(); j++) {
					if (RunGoalsList[j] == goal) {
						found = true;
						break;
					}
				}
				if (found == false) {
					break;
				}
			}
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
			num_collisions = 0;
			// log assignment algorithm type
			log << "/" << it->first << std::endl;
			std::vector<Point> GoalsList = RunGoalsList;
			time_t start_time = time(NULL);

			int n = NUM_ROBOTS;
			NUM_CUR_GOALS = n;
#if (INJECT_METHOD == 2)
			ros::Timer timer = nh.createTimer(ros::Duration(FREQUENCY), timerCallback);
			timer.start();
#endif
			while (GoalsList.size() > 0) {
#if (INJECT_METHOD == 1)
				NUM_CUR_GOALS = NUM_CUR_GOALS + 1;
#endif
				NUM_CUR_GOALS = std::min(NUM_CUR_GOALS,(int)GoalsList.size());

				Test t;

				// retrieve current robots locations
				for (int i = 0; i < n; i++) {
				  std::pair<std::pair<int,int>,double> pair = getRobotLocation(i);
				  t.starts.push_back(pair.first);
				}

				// retrieve the next goal locations
				for (int i = 0; i < NUM_CUR_GOALS; i++) {
				  t.targets.push_back(GoalsList.back());
				  GoalsList.pop_back();
				}

				ROS_INFO("Trying to run task assignment algorithm");
				// solve the task assignment problem
				std::vector<Edge> answer =
						((AssignmentAlgo*)(it->second))(t);
				ROS_INFO("Success !!!");

				// put the targets on the stage map
				for (int i=0; i < answer.size(); i++) {
					Point p = t.targets[answer[i].second.second];
					geometry_msgs::Pose2D msg;
					msg.x = p.first;
					msg.y = p.second;
					msg.theta = 0;
					PosePublishersList[i].publish(msg);
				}

				// log the task assignment chosen & and the time before performing it
				log << "Time " << time(NULL) - start_time << std::endl;

				log << "Robots Locations";
				for (int i = 0; i < n; i++) {
					log << " " << i << "-(" << t.starts[i].first << "," << t.starts[i].second << ")";
				}
				log << std::endl;

				log << "Target Locations: ";
				for (int i = 0; i < NUM_CUR_GOALS; i++) {
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

					moveRobot(WalkClientList[robot_index],
							t.targets[target_index]);
				}
				log << std::endl;

				bool finished = false;
				robot_finished = -1;

				ros::spinOnce();
				while ((!finished) && (!shouldAbort)) {
					for (int i=0; i< NUM_ROBOTS; i++) {
						if (WalkClientList[i]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
							ROS_INFO("Robot %d has reached the goal!", i);
							robot_finished = i;
							finished = true;
						}
					}
					ros::spinOnce();
				}
				shouldAbort = false;

				if (finished) {
					ROS_INFO("FINISHED");
					log << "Robot " << robot_finished << " finished At time " << time(NULL) - start_time << std::endl;
					NUM_CUR_GOALS = NUM_CUR_GOALS - 1;
				}
				else {
					ROS_INFO("ABORT");
					log << "Assignment aborted At time " << time(NULL) - start_time << std::endl;
				}
				ROS_INFO("NUM_CUR_GOALS %d", NUM_CUR_GOALS);
				ROS_INFO("GoalsList size %d", GoalsList.size());
				if ((GoalsList.size() == 0) && (NUM_CUR_GOALS <= 4)) {
#if (INJECT_METHOD == 2)
					timer.stop();
#endif
					bool robots_state[NUM_ROBOTS];
					memset(robots_state,false,NUM_ROBOTS);
					robots_state[robot_finished] = true;
					while (NUM_CUR_GOALS > 0) {
						for (int i=0; i< NUM_ROBOTS; i++) {
							if ((WalkClientList[i]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) \
									&& (robots_state[i] == false)){
								ROS_INFO("Robot %d has reached the goal!", i);
								log << "Robot " << i << " finished At time " << time(NULL) - start_time << std::endl;
								robots_state[i] = true;
								NUM_CUR_GOALS--;
							}
						}
					}

					break;
				}

				for (int i = 0; i < NUM_ROBOTS; i++) {
					WalkClientList[i]->cancelAllGoals();
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
			log << "number of collisions is " << num_collisions << std::endl;
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
	online_scram::WalkGoal goal;
	goal.x = location.first;
	goal.y = location.second;
	goal.c = care_collision;
	ros::Rate loopRate(10);

	ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

}
