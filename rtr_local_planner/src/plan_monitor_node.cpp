#include "ros/publisher.h"
#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sstream>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <vector>

typedef struct pos {
  double x, y, yaw;
} pos;

bool amcl_started = false;
pos currentPos;
pos prevPos;
pos goal;

bool newGoalAvailable = false;

double dYaw, dDist;

double getTargetYaw(pos target, pos pose) {
  pos error;
  error.x = target.x - pose.x;
  error.y = target.y - pose.y;

  if (error.x == 0 && error.y == 0) {
    return pose.yaw;
  } else {
    return std::atan2(error.y, error.x);
  }
}

double getTargetDist(pos target, pos pose) {
  return std::sqrt((target.x - pose.x) * (target.x - pose.x) +
                   (target.y - pose.y) * (target.y - pose.y));
}

bool getDerivatives() {
  if (amcl_started && newGoalAvailable) {
    double prevYaw = getTargetYaw(goal, prevPos);
    double currentYaw = getTargetYaw(goal, currentPos);
    dYaw = currentYaw - prevYaw;

    double prevDist = getTargetDist(goal, prevPos);
    double currentDist = getTargetDist(goal, currentPos);
    dDist = currentDist - prevDist;

    return true;
  }
  return false;
}

void goalCallback(const geometry_msgs::PoseStamped::Ptr &msg) {

  newGoalAvailable = true;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
}

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
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "pan_monitor");

  ros::NodeHandle n;

  ros::Subscriber amcl_sub = n.subscribe("amcl_pose", 100, amclCallback);
  ros::Subscriber goal_sub = n.subscribe("next_goal", 100, goalCallback);

  ros::Publisher pub = n.advertise<std_msgs::String>("plan_monitor", 10);

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    if (getDerivatives()) {

      if (dYaw > 0) {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "[WARN]Robot seems to be turning away from the goal-[deviation="
           << dYaw << "]";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
      }

      if (dDist > 0) {
        std_msgs::String msg;

        std::stringstream ss;

        ss << "[WARN]Robot seems to be moving away from the goal-[deviation="
           << dDist << "]";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}