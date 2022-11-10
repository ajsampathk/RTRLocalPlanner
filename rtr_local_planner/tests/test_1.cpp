#include "rtr_local_planner/rtr_local_planner_util.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace rtr_local_planner;

class BasicTestSuite : public ::testing::Test {
public:
  BasicTestSuite() {}
  ~BasicTestSuite() {}
};

TEST_F(BasicTestSuite, distance_null) {
  RTRLocalPLannerHelpers rtr;
  pos target;
  pos current;

  target.x = 0;
  target.y = 0;
  current.x = 0;
  current.y = 0;
  double d = rtr.euclideanDistance(target, current);
  ASSERT_EQ(d, 0) << "Distance should be 0";
}

TEST_F(BasicTestSuite, distance_0) {
  RTRLocalPLannerHelpers rtr;
  pos target;
  pos current;

  target.x = 1;
  target.y = 1;
  current.x = 1;
  current.y = 1;
  double d = rtr.euclideanDistance(target, current);
  ASSERT_EQ(d, 0) << "Distance should be 0";
}

TEST_F(BasicTestSuite, distance_1) {
  RTRLocalPLannerHelpers rtr;
  pos target;
  pos current;

  target.x = 1;
  target.y = 0;
  current.x = 0;
  current.y = 0;
  double d = rtr.euclideanDistance(target, current);
  ASSERT_EQ(d, 1) << "Distance should be 1";
}

TEST_F(BasicTestSuite, yaw_no_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(25));
  ASSERT_EQ(yaw, RAD(25)) << "Yaw should be 25";
}

TEST_F(BasicTestSuite, yaw_cw_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(225));
  double exp = RAD(225) - RAD(360);
  ASSERT_EQ(yaw, exp) << "Yaw should be -135 deg";
}

TEST_F(BasicTestSuite, yaw_ccw_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(-225));
  double exp = RAD(360) - RAD(225);
  ASSERT_EQ(yaw, exp) << "Yaw should be +135 deg";
}

int main(int argc, char **argv) {

  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  return res;
}
