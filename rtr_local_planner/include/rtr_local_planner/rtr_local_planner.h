
#ifndef RTR_LOCAL_PLANNER_ROS_H_
#define RTR_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// time
#include <time.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

// tf2
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// other
#include <array>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

// helper functions
#include "rtr_local_planner/rtr_local_planner_util.h"

// macros
#define KP_LINEAR 0.75
#define KP_ANGULAR 0.3
#define INDEX_JUMP 5
#define DISTANCE_THRESHOLD 0.1
#define YAW_THRESHOLD_DEG 10

namespace rtr_local_planner {

class RTRLocalPlanner : public nav_core::BaseLocalPlanner {

public:
  RTRLocalPlanner();

  RTRLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  ~RTRLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS *costmap_ros_;
  tf2_ros::Buffer *tf_;

  // topics & services
  ros::Subscriber amcl_sub;
  ros::Publisher path_pub;

  // position data
  pos currentPos;
  pos goalPos;
  pos goalError;

  // path plan data
  int planLength;
  int pathIndex;

  RTRLocalPLannerHelpers rh;

  // msg variables
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::Twist cmd;
  geometry_msgs::PoseStamped goalPose;

  // flags
  bool amcl_started;
  bool goal_reached_;
  bool initialized_;

  // functions
  void setVelocity(double linear, double angular);

  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);

  void updateError();

  void updateGoal();
};
}; // namespace rtr_local_planner

#endif
