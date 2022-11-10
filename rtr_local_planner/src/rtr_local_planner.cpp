
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

void RTRLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {

    costmap_ros_ = costmap_ros;
    tf_ = tf;

    ros::NodeHandle gn;

    amcl_sub =
        gn.subscribe("amcl_pose", 100, &RTRLocalPlanner::amclCallback, this);
    path_pub = gn.advertise<geometry_msgs::PoseStamped>("next_goal", 10);

    amcl_started = 0;

    initialized_ = true;

  } else {
    ROS_WARN("Planner already initialized");
  }
}

bool RTRLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &global_plan) {

  if (!initialized_) {
    ROS_ERROR("Planner not initialized");
    return false;
  }

  pathIndex = 1;

  plan = global_plan;
  planLength = (plan).size() - 1;
  updateGoal();

  ROS_INFO("Recieved Plan Size:%d", planLength);

  goal_reached_ = false;

  return true;
}

bool RTRLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {

  if (!initialized_) {
    ROS_ERROR("Planner not inititalized");
    return false;
  }

  if (planLength != 0) {

    double final_yaw = 0;
    double yaw_diff = 0;

    updateError();

    if (goalError.distance < DISTANCE_THRESHOLD) {

      if (pathIndex < planLength) {

        if ((planLength - pathIndex) < INDEX_JUMP) {
          pathIndex = planLength;
        } else {
          pathIndex += INDEX_JUMP;
        }
        updateGoal();

      } else {

        ROS_INFO("Goal Reached");

        final_yaw = tf2::getYaw(plan[planLength].pose.orientation);
        yaw_diff = final_yaw - currentPos.yaw;

        ROS_INFO("Final Orientation:%f", final_yaw);
        ROS_INFO("Current Orientation:%f", currentPos.yaw);

        yaw_diff = directionalYaw(yaw_diff);

        ROS_INFO("Yaw Difference:%f", yaw_diff);

        if (fabs(yaw_diff) > RAD(YAW_THRESHOLD_DEG)) {
          setVelocity(0, yaw_diff * 0.5);
        } else {
          setVelocity(0, 0);
          goal_reached_ = true;
        }
      }

    } else {

      if (fabs(goalError.yaw) > RAD(YAW_THRESHOLD_DEG)) {
        ROS_INFO("Rotating-(YawError):%f", goalError.yaw);
        setVelocity(0, goalError.yaw);

      } else {
        ROS_INFO("Translating-(Error):%f", goalError.distance);
        setVelocity(goalError.distance, 0);
      }
    }
    ROS_INFO("pathIndex:%d", pathIndex);
  }

  cmd_vel = cmd;

  return true;
}

bool RTRLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("Planner not initialized");
    return false;
  }
  return goal_reached_;
}

void RTRLocalPlanner::amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg) {

  pos previousPos;

  if (amcl_started) {
    previousPos.x = currentPos.x;
    previousPos.y = currentPos.y;
  }

  currentPos.x = msg->pose.pose.position.x;
  currentPos.y = msg->pose.pose.position.y;
  currentPos.yaw = tf2::getYaw(msg->pose.pose.orientation);
  updateError();

  if (amcl_started) {
    currentPos.distance += euclideanDistance(currentPos, previousPos);
  }

  amcl_started = 1;
}

void RTRLocalPlanner::setVelocity(double linear, double angular) {
  cmd.linear.x = linear * KP_LINEAR;
  cmd.linear.y = 0;
  cmd.angular.z = angular * KP_ANGULAR;
}

void RTRLocalPlanner::updateGoal() {

  goalPos.x = plan[pathIndex].pose.position.x;
  goalPos.y = plan[pathIndex].pose.position.y;

  goalPose.header.frame_id = "map";

  goalPose.header.stamp = ros::Time::now();

  goalPose.pose = plan[pathIndex].pose;

  path_pub.publish(goalPose);
}

pos RTRLocalPlanner::getError(pos goalPos, pos currentPos) {

  double yaw;
  pos error;
  error.x = (goalPos.x - currentPos.x);
  error.y = (goalPos.y - currentPos.y);

  if (error.y == 0 && error.x == 0) {
    yaw = currentPos.yaw;
  } else {
    yaw = std::atan2(error.y, error.x);
  }

  error.distance = std::sqrt(error.x * error.x + error.y * error.y);
  error.yaw = yaw - currentPos.yaw;

  error.yaw = directionalYaw(error.yaw);

  return error;
}

void RTRLocalPlanner::updateError() {

  goalError = getError(goalPos, currentPos);

  ROS_INFO("Distance to goal:%f", goalError.distance);
  ROS_INFO("Yaw to Goal:%f", goalError.yaw);
}

double RTRLocalPlanner::directionalYaw(double yaw) {
  if (yaw > RAD(180)) {
    yaw -= RAD(360);
  }
  if (yaw < RAD(-180)) {
    yaw += RAD(360);
  }
  return yaw;
}

double RTRLocalPlanner::euclideanDistance(pos target, pos current) {
  pos error;
  error.x = target.x - current.x;
  error.y = target.y - current.y;
  return std::sqrt(error.x * error.x + error.y * error.y);
}

} // namespace rtr_local_planner