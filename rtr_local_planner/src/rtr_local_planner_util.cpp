#include "rtr_local_planner/rtr_local_planner_util.h"

namespace rtr_local_planner {

RTRLocalPLannerHelpers::RTRLocalPLannerHelpers() {}
RTRLocalPLannerHelpers::~RTRLocalPLannerHelpers() {}

double RTRLocalPLannerHelpers::euclideanDistance(pos target, pos current) {

  pos error;
  error.x = target.x - current.x;
  error.y = target.y - current.y;
  return std::sqrt(error.x * error.x + error.y * error.y);
}

double RTRLocalPLannerHelpers::directionalYaw(double yaw) {
  if (yaw > RAD(180)) {
    yaw -= RAD(360);
  }
  if (yaw < RAD(-180)) {
    yaw += RAD(360);
  }
  return yaw;
}

pos RTRLocalPLannerHelpers::getError(pos goalPos, pos currentPos) {

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

}; // namespace rtr_local_planner