#ifndef RTR_LOCAL_PLANNER_UTIL_H_
#define RTR_LOCAL_PLANNER_UTIL_H_

#include <ros/ros.h>

// useful macro
#define RAD(deg) 0.0174532925 * deg

namespace rtr_local_planner {

// struct to store position values
typedef struct pos {
  double x, y, yaw, distance;
} pos;

class RTRLocalPLannerHelpers {

public:
  // empty constructor
  RTRLocalPLannerHelpers();
  ~RTRLocalPLannerHelpers();

  // function to calculate the minimized yaw to target
  double directionalYaw(double yaw);

  // function to calculate the distance between target and current poses
  double euclideanDistance(pos target, pos current);

  // function to calculate the current pose to goal pose errors
  pos getError(pos goalPos, pos currentPos);
};

}; // namespace rtr_local_planner

#endif