#ifndef RTR_LOCAL_PLANNER_H_
#define RTR_LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

using namespace std;

namespace rtr_local_planner {

class RTRLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  RTRLocalPlanner();
  RTRLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  ~RTRLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS *costmap_ros_;
  tf2_ros::Buffer *tf_;
  bool initialized_;
};
}; // namespace rtr_local_planner

#endif