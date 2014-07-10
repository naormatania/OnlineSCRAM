#include "straight_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(straight_planner::StraightPlanner, nav_core::BaseGlobalPlanner)

namespace straight_planner {

	StraightPlanner::StraightPlanner()
	: costmap_ros_(NULL), initialized_(false){}

	StraightPlanner::StraightPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	: costmap_ros_(NULL), initialized_(false){
		initialize(name, costmap_ros);
	}

	void StraightPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		ROS_WARN("Initializing");
		if(!initialized_){
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();

			ros::NodeHandle private_nh("~/" + name);
			private_nh.param("step_size", step_size_, costmap_->getResolution());
			private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
			// world_model_ = new base_local_planner::CostmapModel(*costmap_);

			initialized_ = true;
			ROS_WARN("Initialized");
		}
		else
			ROS_WARN("This planner has already been initialized... doing nothing");
		costmap_ros_->resetLayers();
	}

	//we need to take the footprint of the robot into account when we calculate cost to obstacles
	double StraightPlanner::footprintCost(double x_i, double y_i, double theta_i){
		if(!initialized_){
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return -1.0;
		}

		std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
		//if we have no footprint... do nothing
		if(footprint.size() < 3)
			return -1.0;

		//check if the footprint is legal
		// double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
		// return footprint_cost;
		return 0;
	}


	bool StraightPlanner::makePlan(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		ROS_WARN("Here we are");
		if(!initialized_){
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return false;
		}

		ROS_WARN("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

		plan.clear();
		costmap_ = costmap_ros_->getCostmap();

		if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
			ROS_WARN("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
			costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
			return false;
		}

		tf::Stamped<tf::Pose> goal_tf;
		tf::Stamped<tf::Pose> start_tf;

		poseStampedMsgToTF(goal,goal_tf);
		poseStampedMsgToTF(start,start_tf);

		double useless_pitch, useless_roll, goal_yaw, start_yaw;
		start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
		// goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

		//we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;

		double diff_x = goal_x - start_x;
		double diff_y = goal_y - start_y;
		// double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);
		double target_yaw = atan2(diff_y, diff_x);
		// Convert the Euler angle to quaternion
		double radians = target_yaw * (M_PI/180);
		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(radians);

		double target_x = goal_x;
		double target_y = goal_y;

		double scale = 0;
		plan.push_back(start);
		while(start_x != target_x)
		{
			target_x = start_x + scale * diff_x;
			target_y = start_y + scale * diff_y;

			geometry_msgs::PoseStamped new_goal = goal;

			new_goal.pose.position.x = target_x;
			new_goal.pose.position.y = target_y;

			new_goal.pose.orientation.x = goal_quat.x();
			new_goal.pose.orientation.y = goal_quat.y();
			new_goal.pose.orientation.z = goal_quat.z();
			new_goal.pose.orientation.w = goal_quat.w();

			plan.push_back(new_goal);

			scale += 0.1;

		}

		ROS_WARN("Reset Layers");
		costmap_ros_->resetLayers();
		ROS_WARN("Return new goal");
		return true;
	}

};

/**
 * solution for stageros.cpp
 *
#include <geometry_msgs/Pose2D.h>

#define POSE "pose"

std::vector<ros::Subscriber> pose_subs_;

void
StageNode::poseReceived(int idx, const boost::shared_ptr<geometry_msgs::Pose2D const>& msg)
{
  boost::mutex::scoped_lock lock(msg_lock);
    Stg::Pose pose;
    pose.x = msg->x;
    pose.y = msg->y;
    pose.z =0;
    pose.a = msg->theta;
    this->positionmodels[idx]->SetPose(pose);
}


pose_subs_.push_back(n_.subscribe<geometry_msgs::Pose2D>(mapName(POSE,r), 10, boost::bind(&StageNode::poseReceived, this, r, _1)));



 or:
 /robot_0/move_base_node/local_costmap
 /robot_0/move_base_node/local_costmap
  *
 */
