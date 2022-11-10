#include "ros/publisher.h"
#include "ros/ros.h"
#include "rtr_local_planner/rtr_local_planner_util.h"

#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <std_msgs/String.h>
#include <tf2/utils.h>

#define D_YAW_THRESHOLD 0.1
#define D_DIST_THREDSHOLD 0.1
#define D_COUNT_THRESHOLD 10

bool amcl_started = false;
bool amclAvailable = false;

using namespace rtr_local_planner;

RTRLocalPLannerHelpers rh;
pos currentPos;
pos prevPos;
pos goal;

bool newGoalAvailable = false;

double dYaw, dDist;

ros::Publisher pub;

// calculate the current apparent velocities of the robot
bool getDerivatives() {
  if (amclAvailable && newGoalAvailable) {
    pos prevError = rh.getError(goal, prevPos);
    pos currentError = rh.getError(goal, currentPos);

    dYaw = currentPos.yaw - prevPos.yaw;
    dDist = currentPos.distance - prevPos.distance;
    amclAvailable = false;
    return true;
  }
  return false;
}

// set new goal
void goalCallback(const geometry_msgs::PoseStamped::Ptr &msg) {

  newGoalAvailable = true;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
}

// recieve amcl localized position of the robot
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg) {

  if (amcl_started) {
    prevPos.x = currentPos.x;
    prevPos.y = currentPos.y;
    prevPos.yaw = currentPos.yaw;
  }
  currentPos.x = msg->pose.pose.position.x;
  currentPos.y = msg->pose.pose.position.y;
  currentPos.yaw = tf2::getYaw(msg->pose.pose.orientation);

  amcl_started = true;
  amclAvailable = true;
}

// publish monitor message
void publishMessage(const std::stringstream *outs) {
  std_msgs::String msg;
  msg.data = outs->str();
  ROS_INFO("%s", msg.data.c_str());
  pub.publish(msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "plan_monitor");

  ros::NodeHandle n;

  ros::Subscriber amcl_sub = n.subscribe("amcl_pose", 100, amclCallback);
  ros::Subscriber goal_sub = n.subscribe("next_goal", 100, goalCallback);

  pub = n.advertise<std_msgs::String>("plan_monitor", 10);

  ros::Rate loop_rate(10);

  int dYaw_count = 0;
  int dDist_count = 0;

  while (ros::ok()) {

    if (!newGoalAvailable)
      ROS_INFO("Waiting for goal");
    else {

      if (getDerivatives()) {

        // if current angular velocity is away from the goal it is a yaw
        // deviation
        if (dYaw > D_YAW_THRESHOLD) {

          std::stringstream ss;
          ss << "[WARN]Yaw deviation=" << dYaw;
          publishMessage(&ss);
          dYaw_count++;
        } else {
          dYaw_count = 0;
        }

        // if current linear velocity is away from the goal it is a yaw
        // deviation
        if (dDist > D_DIST_THREDSHOLD) {
          std::stringstream ss;
          ss << "[WARN]Dist deviation=" << dDist;
          publishMessage(&ss);
          dDist_count++;
        } else {
          dDist_count = 0;
        }
      }
      // if the velocities are continously away from the goal it is a critical
      // deviation
      if (dYaw_count > D_COUNT_THRESHOLD || dDist_count > D_COUNT_THRESHOLD) {
        std::stringstream ss;
        ss << "[CRITICAL]Robot seems to be deviating from the goal";
        publishMessage(&ss);
      }
    }
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}