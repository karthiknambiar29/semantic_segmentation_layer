// Copyright (c) 2026, robot.com
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "semantic_segmentation_layer/camera_projection.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using semantic_segmentation_layer::CameraProjection;

namespace
{
sensor_msgs::msg::CameraInfo makeInfo(uint32_t w = 100, uint32_t h = 100, double f = 100.0)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {f, 0.0, w / 2.0, 0.0, f, h / 2.0, 0.0, 0.0, 1.0};
  info.p = {f, 0.0, w / 2.0, 0.0, 0.0, f, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}
}  // namespace

TEST(CameraProjection, RejectsEmptyCameraInfo)
{
  CameraProjection proj;
  EXPECT_FALSE(proj.fromCameraInfo(sensor_msgs::msg::CameraInfo{}));
  EXPECT_FALSE(proj.valid());
}

TEST(CameraProjection, ProjectsOpticalPointToPrincipalPoint)
{
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(makeInfo()));
  double u, v;
  ASSERT_TRUE(proj.project(0.0, 0.0, 3.0, u, v));  // straight ahead in optical frame
  EXPECT_NEAR(u, 50.0, 1e-6);
  EXPECT_NEAR(v, 50.0, 1e-6);
}

TEST(CameraProjection, ProjectsOffAxisOpticalPoint)
{
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(makeInfo()));
  double u, v;
  ASSERT_TRUE(proj.project(0.5, 0.0, 2.0, u, v));  // x right, z fwd
  EXPECT_NEAR(u, 100.0 * 0.5 / 2.0 + 50.0, 1e-6);  // -> 75
  EXPECT_NEAR(v, 50.0, 1e-6);
}

TEST(CameraProjection, RejectsPointsBehindImagePlane)
{
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(makeInfo()));
  double u, v;
  EXPECT_FALSE(proj.project(0.0, 0.0, -1.0, u, v));
  EXPECT_FALSE(proj.project(0.0, 0.0, 0.0, u, v));
}

TEST(CameraProjection, RejectsNonFinitePoints)
{
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(makeInfo()));
  double u, v;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(proj.project(nan, 0.0, 1.0, u, v));
  EXPECT_FALSE(proj.project(0.0, 0.0, nan, u, v));
  EXPECT_FALSE(proj.project(0.0, inf, 1.0, u, v));
}

// The physical camera frame (rgb_camera_frame) is +x fwd / +y left / +z up.
// A ROS optical frame is +x right / +y down / +z fwd. These are NOT the same;
// treating one as the other silently mis-projects every point.
TEST(CameraProjection, PhysicalFrameIsNotOpticalFrame)
{
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(makeInfo()));

  // Point 3 m directly in front of the physical camera: +x forward.
  double ox, oy, oz;
  CameraProjection::opticalFromPhysical(3.0, 0.0, 0.0, ox, oy, oz);
  EXPECT_NEAR(ox, 0.0, 1e-9);
  EXPECT_NEAR(oy, 0.0, 1e-9);
  EXPECT_NEAR(oz, 3.0, 1e-9);

  double u, v;
  ASSERT_TRUE(proj.project(ox, oy, oz, u, v));
  EXPECT_NEAR(u, 50.0, 1e-6);
  EXPECT_NEAR(v, 50.0, 1e-6);

  // If rgb_camera_frame were (wrongly) fed straight to project(), the same
  // physical point has z = 0 (its +x) and would be rejected as behind the plane.
  EXPECT_FALSE(proj.project(3.0, 0.0, 0.0, u, v));

  // A point that is really overhead (physical +z up) must not read as "forward":
  // its optical z is 0 (on the image plane), so it is not projectable.
  CameraProjection::opticalFromPhysical(0.0, 0.0, 5.0, ox, oy, oz);
  EXPECT_DOUBLE_EQ(oz, 0.0);
  EXPECT_FALSE(proj.project(ox, oy, oz, u, v));
}

TEST(CameraProjection, AppliesRadialDistortionWhenPresent)
{
  auto info = makeInfo();
  info.d = {0.2, 0.0, 0.0, 0.0, 0.0};  // barrel distortion
  CameraProjection proj;
  ASSERT_TRUE(proj.fromCameraInfo(info));
  double u_dist, v_dist;
  ASSERT_TRUE(proj.project(0.5, 0.0, 2.0, u_dist, v_dist));

  CameraProjection undistorted;
  ASSERT_TRUE(undistorted.fromCameraInfo(makeInfo()));
  double u_plain, v_plain;
  ASSERT_TRUE(undistorted.project(0.5, 0.0, 2.0, u_plain, v_plain));

  EXPECT_GT(std::abs(u_dist - 50.0), std::abs(u_plain - 50.0));  // pushed outward
}
