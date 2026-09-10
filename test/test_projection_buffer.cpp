// Copyright (c) 2026, robot.com
// Licensed under the Apache License, Version 2.0.
//
// End-to-end coverage of the LiDAR-projection buffering path:
//   /lidar/points -> TF -> intrinsic projection -> mask sample -> tile map.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "semantic_segmentation_layer/segmentation_buffer.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"

using semantic_segmentation_layer::SegmentationBuffer;

namespace
{

constexpr uint32_t kW = 100;
constexpr uint32_t kH = 100;
constexpr double kF = 100.0;              // fx = fy
constexpr double kRes = 0.05;             // costmap resolution [m]
const rclcpp::Time kStamp(12345, 0, RCL_ROS_TIME);

sensor_msgs::msg::CameraInfo makeCameraInfo(const std::string & frame)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame;
  info.width = kW;
  info.height = kH;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {kF, 0.0, kW / 2.0, 0.0, kF, kH / 2.0, 0.0, 0.0, 1.0};
  info.p = {kF, 0.0, kW / 2.0, 0.0, 0.0, kF, kH / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

// mono8 mask, all 0 (obstacle) except the pixels in `traversable` set to 255.
sensor_msgs::msg::Image makeMask(const std::vector<std::pair<int, int>> & traversable)
{
  sensor_msgs::msg::Image img;
  img.header.frame_id = "rgb_camera_frame";
  img.header.stamp = kStamp;
  img.height = kH;
  img.width = kW;
  img.encoding = "mono8";
  img.step = kW;
  img.data.assign(static_cast<size_t>(kW) * kH, 0);
  for (const auto & uv : traversable) {
    img.data[uv.second * kW + uv.first] = 255;
  }
  return img;
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const std::string & frame, const std::vector<std::array<float, 3>> & pts)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = kStamp;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (const auto & p : pts) {
    *ix = p[0];
    *iy = p[1];
    *iz = p[2];
    ++ix;
    ++iy;
    ++iz;
  }
  return cloud;
}

geometry_msgs::msg::TransformStamped tf(
  const std::string & parent, const std::string & child, double x, double y, double z,
  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  return t;
}

class ProjectionBufferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_projection_buffer");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
  }

  // camera_optical_frame empty -> physical(rgb_camera_frame) + optical swap path.
  std::unique_ptr<SegmentationBuffer> makeBuffer(const std::string & optical_frame = "")
  {
    std::unordered_map<std::string, CostHeuristicParams> cost_map{
      {"traversable", CostHeuristicParams{0, 0, 0, 0, false}},
      {"obstacle", CostHeuristicParams{254, 254, 0, 0, false}}};
    std::unordered_map<std::string, std::vector<std::string>> type_to_names{
      {"traversable", {"traversable"}}, {"obstacle", {"obstacle"}}};

    auto buf = std::make_unique<SegmentationBuffer>(
      node_, "lidar_cam", std::vector<std::string>{"traversable", "obstacle"}, cost_map,
      type_to_names, /*keep_time*/ 0.0, /*update_rate*/ 0.0, /*max_dist*/ 100.0, /*min_dist*/ 0.0,
      *tf_buffer_, /*global_frame*/ "odom", /*sensor_frame*/ "",
      tf2::durationFromSec(0.0), kRes, /*decay*/ 5.0, /*visualize*/ false,
      /*use_cost_selection*/ false, /*project_pointcloud*/ true, optical_frame);
    buf->createSegmentationCostMultimapFromIds({{"traversable", 255}, {"obstacle", 0}});
    return buf;
  }

  void addCoLocatedFrames()
  {
    tf_buffer_->setTransform(tf("odom", "lidar_frame", 0, 0, 0), "test", true);
    tf_buffer_->setTransform(tf("odom", "rgb_camera_frame", 0, 0, 0), "test", true);
  }

  // Count observations across the whole tile map, and the class of the first tile.
  void inspectTileMap(SegmentationBuffer & buf, size_t & n_tiles, int & first_class)
  {
    n_tiles = 0;
    first_class = -2;
    auto map = buf.getSegmentationTileMap();
    map->lock();
    for (auto & tile : *map) {
      ++n_tiles;
      if (first_class == -2) {
        first_class = tile.second.getClassId();
      }
    }
    map->unlock();
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

// Physical point (+x fwd, +y left): (2, -0.5, 0) -> optical (0.5, 0, 2) -> pixel (75, 50).
TEST_F(ProjectionBufferTest, PointOnTraversablePixelBuffersTraversableClass)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({{75, 50}});
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud("lidar_frame", {{2.0f, -0.5f, 0.0f}});

  buf->bufferSegmentation(cloud, mask, conf);

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 1u);
  EXPECT_EQ(first_class, 255);
}

// Same geometry mirrored: (2, 0.5, 0) -> optical (-0.5, 0, 2) -> pixel (25, 50) == 0.
TEST_F(ProjectionBufferTest, PointOnObstaclePixelBuffersObstacleClass)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({{75, 50}});  // (25,50) stays 0
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud("lidar_frame", {{2.0f, 0.5f, 0.0f}});

  buf->bufferSegmentation(cloud, mask, conf);

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 1u);
  EXPECT_EQ(first_class, 0);
}

TEST_F(ProjectionBufferTest, RejectsPointBehindCameraOutsideImageAndNonFinite)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({});
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  auto cloud = makeCloud(
    "lidar_frame", {
      {-2.0f, 0.0f, 0.0f}, // behind camera (physical -x)
      {0.1f, -5.0f, 0.0f}, // way off to the side -> u far outside image
      {nan, 0.0f, 1.0f},   // NaN
      {1.0f, 0.0f, inf},   // Inf
    });

  buf->bufferSegmentation(cloud, mask, conf);

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 0u);
}

TEST_F(ProjectionBufferTest, MultiplePointsSamePixelAndTileYieldOneObservation)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({{75, 50}});
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  // optical x/z = (-py)/px = 0.25 for both -> both project to pixel (75, 50).
  // Both global positions fall in tile x=floor(2.0x/0.05)=40, y=floor(-0.50x/0.05)=-11.
  auto cloud = makeCloud(
    "lidar_frame", {
      {2.01f, -0.5025f, 0.0f},
      {2.03f, -0.5075f, 0.0f},
    });

  buf->bufferSegmentation(cloud, mask, conf);

  auto map = buf->getSegmentationTileMap();
  size_t n_tiles = 0;
  size_t dominant_obs = 0;
  map->lock();
  for (auto & tile : *map) {
    ++n_tiles;
    dominant_obs = tile.second.size();
  }
  map->unlock();
  EXPECT_EQ(n_tiles, 1u);
  EXPECT_EQ(dominant_obs, 1u);  // one observation per tile per frame
}

// odom->rgb_camera_frame rotated +90deg yaw: a point 2 m along odom +y is 2 m
// "forward" for the camera. Only a correct TF lookup puts it at the principal
// point; skipping the rotation would treat it as behind the image plane.
TEST_F(ProjectionBufferTest, UsesTfRotationFromLidarToCameraFrame)
{
  tf_buffer_->setTransform(tf("odom", "lidar_frame", 0, 0, 0), "test", true);
  // yaw +90 deg -> quaternion (0,0,sin45,cos45)
  const double s = std::sin(M_PI / 4.0);
  tf_buffer_->setTransform(tf("odom", "rgb_camera_frame", 0, 0, 0, 0, 0, s, s), "test", true);

  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({{50, 50}});  // principal point is traversable
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud("lidar_frame", {{0.0f, 2.0f, 0.0f}});  // 2 m along odom +y

  buf->bufferSegmentation(cloud, mask, conf);

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 1u);
  EXPECT_EQ(first_class, 255);
}

TEST_F(ProjectionBufferTest, MissingTfDoesNotCrashAndBuffersNothing)
{
  // No transforms added at all.
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  auto mask = makeMask({{75, 50}});
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud("lidar_frame", {{2.0f, -0.5f, 0.0f}});

  EXPECT_NO_THROW(buf->bufferSegmentation(cloud, mask, conf));

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 0u);
}

TEST_F(ProjectionBufferTest, MissingCameraInfoDoesNotCrashAndBuffersNothing)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  // setCameraInfo intentionally NOT called.
  auto mask = makeMask({{75, 50}});
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud("lidar_frame", {{2.0f, -0.5f, 0.0f}});

  EXPECT_NO_THROW(buf->bufferSegmentation(cloud, mask, conf));

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 0u);
}

// Continuous / score mask: value >= 146 -> traversable, <= 110 -> obstacle,
// the 111..145 band is "uncertain" and dropped. Three co-located LiDAR points
// project to three pixels holding 200, 128 and 40.
TEST_F(ProjectionBufferTest, ScoreThresholdMapsMaskValuesToClasses)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  buf->setValueRanges(
    {
      SegmentationValueRange{0, 110, 0},    // obstacle id
      SegmentationValueRange{146, 255, 255}, // traversable id
    });

  auto mask = makeMask({});  // all zero to start
  mask.data[50 * kW + 75] = 200;   // -> traversable
  mask.data[50 * kW + 50] = 128;   // -> uncertain band -> dropped
  mask.data[50 * kW + 25] = 40;    // -> obstacle
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);

  // physical (x fwd, y left): these hit pixels u=75, u=50, u=25 at v=50.
  auto cloud = makeCloud(
    "lidar_frame", {
      {2.0f, -0.5f, 0.0f}, // -> (75, 50) = 200
      {2.0f, 0.0f, 0.0f},  // -> (50, 50) = 128
      {2.0f, 0.5f, 0.0f},  // -> (25, 50) = 40
    });

  buf->bufferSegmentation(cloud, mask, conf);

  auto map = buf->getSegmentationTileMap();
  size_t n_tiles = 0;
  bool saw_traversable = false, saw_obstacle = false;
  map->lock();
  for (auto & tile : *map) {
    ++n_tiles;
    if (tile.second.getClassId() == 255) {saw_traversable = true;}
    if (tile.second.getClassId() == 0) {saw_obstacle = true;}
  }
  map->unlock();
  EXPECT_EQ(n_tiles, 2u);          // the 128 point was dropped
  EXPECT_TRUE(saw_traversable);
  EXPECT_TRUE(saw_obstacle);
}

// Direct-cost mode: the mask value maps straight to a cost, no class binning.
// cost = round(254 * (1 - v/255)):  v=255 -> 0, v=0 -> 254, v=128 -> 127.
TEST_F(ProjectionBufferTest, DirectCostMapsMaskValueToCost)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  buf->setDirectCostMapping(254.0, 0.0, -1, -1);

  auto mask = makeMask({});
  mask.data[50 * kW + 75] = 255;   // -> cost 0
  mask.data[50 * kW + 50] = 0;     // -> cost 254
  mask.data[50 * kW + 25] = 128;   // -> cost 127
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud(
    "lidar_frame", {
      {2.0f, -0.5f, 0.0f}, // (75, 50)
      {2.0f, 0.0f, 0.0f},  // (50, 50)
      {2.0f, 0.5f, 0.0f},  // (25, 50)
    });

  buf->bufferSegmentation(cloud, mask, conf);

  auto map = buf->getSegmentationTileMap();
  std::vector<int> costs;
  map->lock();
  for (auto & tile : *map) {
    costs.push_back(tile.second.getClassId());  // class id == cost in direct mode
  }
  map->unlock();
  std::sort(costs.begin(), costs.end());
  ASSERT_EQ(costs.size(), 3u);
  EXPECT_EQ(costs[0], 0);
  EXPECT_EQ(costs[1], 127);
  EXPECT_EQ(costs[2], 254);
}

// Two points on one tile: the tile reports the worst (highest) cost.
TEST_F(ProjectionBufferTest, DirectCostTileReportsWorstScore)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  buf->setDirectCostMapping(254.0, 0.0, -1, -1);

  auto mask = makeMask({});
  mask.data[50 * kW + 75] = 255;   // free
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  // Both project to (75,50) and land in the same tile (see the multi-point test).
  // First frame: value 255 -> cost 0.
  buf->bufferSegmentation(makeCloud("lidar_frame", {{2.01f, -0.5025f, 0.0f}}), mask, conf);
  // Second frame: same tile now reads value 0 -> cost 254.
  mask.data[50 * kW + 75] = 0;
  buf->bufferSegmentation(makeCloud("lidar_frame", {{2.03f, -0.5075f, 0.0f}}), mask, conf);

  size_t n_tiles;
  int worst;
  inspectTileMap(*buf, n_tiles, worst);
  EXPECT_EQ(n_tiles, 1u);
  EXPECT_EQ(worst, 254);  // worst score sticks until it decays
}

// Values inside the ignore band are treated as unknown and dropped.
TEST_F(ProjectionBufferTest, DirectCostIgnoreBandDropsPixels)
{
  addCoLocatedFrames();
  auto buf = makeBuffer();
  buf->setCameraInfo(makeCameraInfo("rgb_camera_frame"));
  buf->setDirectCostMapping(254.0, 0.0, /*ignore_lo*/ 120, /*ignore_hi*/ 140);

  auto mask = makeMask({});
  mask.data[50 * kW + 75] = 130;   // inside ignore band -> dropped
  mask.data[50 * kW + 25] = 10;    // -> obstacle
  auto conf = mask;
  conf.data.assign(conf.data.size(), 255);
  auto cloud = makeCloud(
    "lidar_frame", {
      {2.0f, -0.5f, 0.0f}, // (75, 50) = 130 -> dropped
      {2.0f, 0.5f, 0.0f},  // (25, 50) = 10  -> kept
    });

  buf->bufferSegmentation(cloud, mask, conf);

  size_t n_tiles;
  int first_class;
  inspectTileMap(*buf, n_tiles, first_class);
  EXPECT_EQ(n_tiles, 1u);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
