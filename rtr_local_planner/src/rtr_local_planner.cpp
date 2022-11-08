#include "rtr_local_planner/rtr_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rtr_local_planner::RTRLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace rtr_local_planner {

RTRLocalPlanner::RTRLocalPlanner()
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

RTRLocalPlanner::RTRLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, tf, costmap_ros);
}

RTRLocalPlanner::~RTRLocalPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in
// ROS Noetic
void RTRLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    initialized_ = true;
  }
}

bool RTRLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return true;
}

bool RTRLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return true;
}

bool RTRLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return false;
}
} // namespace rtr_local_planner