#include "rtr_local_planner/rtr_local_planner_util.h"

#include <gtest/gtest.h>

using namespace rtr_local_planner;

class BasicTestSuite : public ::testing::Test {
public:
  BasicTestSuite() {}
  ~BasicTestSuite() {}
};

// test eucledian distance function with 0 values
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

// test eucledian distance function with same values

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

// test eucledian distance function with different values

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

// test yaw minimizing distance function with default case

TEST_F(BasicTestSuite, yaw_no_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(25));
  ASSERT_EQ(yaw, RAD(25)) << "Yaw should be 25";
}

// test yaw minimizing distance function with CCW case
TEST_F(BasicTestSuite, yaw_cw_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(225));
  double exp = RAD(225) - RAD(360);
  ASSERT_EQ(yaw, exp) << "Yaw should be -135 deg";
}

// test yaw minimizing distance function with CW case
TEST_F(BasicTestSuite, yaw_ccw_change) {
  RTRLocalPLannerHelpers rtr;
  double yaw = rtr.directionalYaw(RAD(-225));
  double exp = RAD(360) - RAD(225);
  ASSERT_EQ(yaw, exp) << "Yaw should be +135 deg";
}

// test all components of error calculating function
TEST_F(BasicTestSuite, pos_error_all) {
  RTRLocalPLannerHelpers rtr;
  pos target;
  pos current;

  target.x = 1;
  target.y = 1;
  current.x = 0;
  current.y = 0;
  pos error = rtr.getError(target, current);
  ASSERT_EQ(error.x, 1) << "x error should be 1";
  ASSERT_EQ(error.y, 1) << "x error should be 1";
  ASSERT_DOUBLE_EQ(error.distance, std::sqrt(2))
      << "distance should be sqrt(2)";
  ASSERT_TRUE(fabs(error.yaw - RAD(45)) < 0.0001) << "Yaw should be 45deg";
}

// test the singularity case of error calculating function
TEST_F(BasicTestSuite, pos_error_yaw_singularity) {
  RTRLocalPLannerHelpers rtr;
  pos target;
  pos current;

  target.x = 0;
  target.y = 0;
  current.x = 0;
  current.y = 0;
  current.yaw = 0.174;
  pos error = rtr.getError(target, current);
  ASSERT_EQ(error.x, 0) << "x error should be 0";
  ASSERT_EQ(error.y, 0) << "x error should be 0";
  ASSERT_DOUBLE_EQ(error.distance, 0) << "distance should be sqrt(2)";
  ASSERT_EQ(error.yaw, 0) << "Yaw should be 0";
}

int main(int argc, char **argv) {

  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  return res;
}
