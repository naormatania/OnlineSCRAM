#ifndef straight_planner_H_
#define straight_planner_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace straight_planner{
	class StraightPlanner : public nav_core::BaseGlobalPlanner {
		public:
			StraightPlanner();
			StraightPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

			bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		private:
			costmap_2d::Costmap2DROS* costmap_ros_;
			double step_size_, min_dist_from_robot_;
			costmap_2d::Costmap2D* costmap_;
			base_local_planner::WorldModel* world_model_;

			double footprintCost(double x_i, double y_i, double theta_i);

			bool initialized_;
};
};
#endif
