#include "rtr_local_planner/rtr_local_planner_util.h"

namespace rtr_local_planner {

// empty constructor
RTRLocalPLannerHelpers::RTRLocalPLannerHelpers() {}
RTRLocalPLannerHelpers::~RTRLocalPLannerHelpers() {}

// calculate eucledian distance
double RTRLocalPLannerHelpers::euclideanDistance(pos target, pos current) {

  pos error;
  error.x = target.x - current.x;
  error.y = target.y - current.y;
  return std::sqrt(error.x * error.x + error.y * error.y);
}

double RTRLocalPLannerHelpers::directionalYaw(double yaw) {

  // Minimize the rotation by turning either:
  if (yaw > RAD(180)) {
    // Clockwise
    yaw -= RAD(360);
  }
  if (yaw < RAD(-180)) {
    // Counter-Clockwise
    yaw += RAD(360);
  }
  return yaw;
}

pos RTRLocalPLannerHelpers::getError(pos goalPos, pos currentPos) {

  double yaw;
  pos error;

  // get component errors
  error.x = (goalPos.x - currentPos.x);
  error.y = (goalPos.y - currentPos.y);

  // avoid singularity
  if (error.y == 0 && error.x == 0) {
    // return the current yaw as target yaw
    yaw = currentPos.yaw;
  } else {
    yaw = std::atan2(error.y, error.x);
  }

  // calculate eucledian distance
  error.distance = std::sqrt(error.x * error.x + error.y * error.y);
  // error in yaw
  error.yaw = yaw - currentPos.yaw;

  // minimize yaw
  error.yaw = directionalYaw(error.yaw);

  return error;
}

}; // namespace rtr_local_planner