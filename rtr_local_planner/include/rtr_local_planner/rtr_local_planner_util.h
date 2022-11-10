#ifndef RTR_LOCAL_PLANNER_UTIL_H_
#define RTR_LOCAL_PLANNER_UTIL_H_

#include <ros/ros.h>

#define RAD(deg) 0.0174532925 * deg

namespace rtr_local_planner {

typedef struct pos {

  double x, y, yaw, distance;
} pos;

class RTRLocalPLannerHelpers {

public:
  RTRLocalPLannerHelpers();
  ~RTRLocalPLannerHelpers();
  double directionalYaw(double yaw);

  double euclideanDistance(pos target, pos current);

  pos getError(pos goalPos, pos currentPos);
};

}; // namespace rtr_local_planner

#endif