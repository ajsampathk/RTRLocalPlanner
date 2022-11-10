
#include "rtr_local_planner/rtr_local_planner.h"
#include <pluginlib/class_list_macros.h>

// export class as plugin
PLUGINLIB_EXPORT_CLASS(rtr_local_planner::RTRLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace rtr_local_planner {

RTRLocalPlanner::RTRLocalPlanner()
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

// constructor to initialize planner
RTRLocalPlanner::RTRLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, tf, costmap_ros);
}

RTRLocalPlanner::~RTRLocalPlanner() {}

// initialize planner
void RTRLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {

    costmap_ros_ = costmap_ros;
    tf_ = tf;

    ros::NodeHandle gn;

    //  Subscribe to amcl localization and publish persuing goal to a topic

    amcl_sub =
        gn.subscribe("amcl_pose", 100, &RTRLocalPlanner::amclCallback, this);
    path_pub = gn.advertise<geometry_msgs::PoseStamped>("next_goal", 10);

    //  set flags
    amcl_started = 0;

    initialized_ = true;

  } else {
    ROS_WARN("Planner already initialized");
  }
}

// Recieve plan from global planner
bool RTRLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &global_plan) {

  if (!initialized_) {
    ROS_ERROR("Planner not initialized");
    return false;
  }

  //   reset current pathIndex
  pathIndex = 1;

  plan = global_plan;
  planLength = (plan).size() - 1;
  updateGoal();

  ROS_INFO("Recieved Plan Size:%d", planLength);

  //  reset goal status flag
  goal_reached_ = false;

  return true;
}

bool RTRLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {

  if (!initialized_) {
    ROS_ERROR("Planner not inititalized");
    return false;
  }

  // Check if there is a valid plan
  if (planLength != 0) {

    double final_yaw = 0;
    double yaw_diff = 0;

    // calculate error to next goal point
    updateError();

    // check if robot is within goal distance threshold
    if (goalError.distance < DISTANCE_THRESHOLD) {

      // check if it is not the last point in the plan
      if (pathIndex < planLength) {

        /*
            INDEX_JUMP is used to select the next goal point in the plan that
           will be above the goal threshold. Note that if the INDEX_JUMP is set
           to 1, the next goal point will be too close and the proportional
           controller will supply a very small velocity to the robot which will
           result in the robot not moving and causes move_base to start recovery
           behaviours.
        */
        if ((planLength - pathIndex) < INDEX_JUMP) {
          pathIndex = planLength;
        } else {
          pathIndex += INDEX_JUMP;
        }

        // get next goal point in plan with the new pathIndex
        updateGoal();

      } else {

        // if it is the last point in the plan

        ROS_INFO("Goal Reached");

        //              ROTATE
        // The robot needs to rotate to the goal orientation.

        final_yaw = tf2::getYaw(plan[planLength].pose.orientation);
        yaw_diff = final_yaw - currentPos.yaw;

        ROS_INFO("Final Orientation:%f", final_yaw);
        ROS_INFO("Current Orientation:%f", currentPos.yaw);

        // Get the suitable direction to rotate (CW or CCW)
        yaw_diff = rh.directionalYaw(yaw_diff);

        ROS_INFO("Yaw Difference:%f", yaw_diff);

        /*
            YAW_THRESHOLD_DEG is set to 10 degrees, any tolerance less than this
            results in move_base starting recovery behaviours with the default
            params used in the turtlebot3_navigation
        */
        if (fabs(yaw_diff) > RAD(YAW_THRESHOLD_DEG)) {
          setVelocity(0, yaw_diff * 0.5);
        } else {
          setVelocity(0, 0);
          goal_reached_ = true;
        }
      }

    } else {

      /*
                  ROTATE
      If the goal is not in line with the robot's orientation, it needs to
      turn to face the goal,

                  TRANSLATE
      Otherwise, if the goal is in line with the robot's orientation, it can
      start to move towards it

      */
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

  // if this is not the first message than store the previous position in a
  // variable
  if (amcl_started) {
    previousPos.x = currentPos.x;
    previousPos.y = currentPos.y;
  }

  currentPos.x = msg->pose.pose.position.x;
  currentPos.y = msg->pose.pose.position.y;
  // get the euler angle yaw of the current position
  currentPos.yaw = tf2::getYaw(msg->pose.pose.orientation);

  // update the error to goal
  updateError();

  if (amcl_started) {
    // calculate distance to goal
    currentPos.distance += rh.euclideanDistance(currentPos, previousPos);
  }

  amcl_started = 1;
}

void RTRLocalPlanner::setVelocity(double linear, double angular) {

  // Use proportional controller to set output velocities
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

  // publish next goal to topic
  path_pub.publish(goalPose);
}

void RTRLocalPlanner::updateError() {

  // calculate error to goal
  goalError = rh.getError(goalPos, currentPos);

  ROS_INFO("Distance to goal:%f", goalError.distance);
  ROS_INFO("Yaw to Goal:%f", goalError.yaw);
}

} // namespace rtr_local_planner