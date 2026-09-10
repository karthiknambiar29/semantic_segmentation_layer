/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2026, robot.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of robot.com nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/
#include "semantic_segmentation_layer/segmentation_buffer.hpp"

#include <algorithm>
#include <chrono>
#include <list>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/convert.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

namespace semantic_segmentation_layer {
SegmentationBuffer::SegmentationBuffer(const nav2_util::LifecycleNode::WeakPtr& parent,
                                       std::string buffer_source, std::vector<std::string> class_types, std::unordered_map<std::string, CostHeuristicParams> class_names_cost_map,
                                       std::unordered_map<std::string, std::vector<std::string>> class_type_to_names,
                                       double observation_keep_time,
                                       double expected_update_rate, double max_lookahead_distance,
                                       double min_lookahead_distance, tf2_ros::Buffer& tf2_buffer,
                                       std::string global_frame, std::string sensor_frame,
                                       tf2::Duration tf_tolerance, double costmap_resolution, double tile_map_decay_time, bool visualize_tile_map,
                                       bool use_cost_selection, bool project_pointcloud,
                                       std::string camera_optical_frame)
  : tf2_buffer_(tf2_buffer)
  , class_types_(class_types)
  , class_names_cost_map_(class_names_cost_map)
  , class_type_to_names_(class_type_to_names)
  , observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time))
  , expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate))
  , global_frame_(global_frame)
  , sensor_frame_(sensor_frame)
  , buffer_source_(buffer_source)
  , sq_max_lookahead_distance_(std::pow(max_lookahead_distance, 2))
  , sq_min_lookahead_distance_(std::pow(min_lookahead_distance, 2))
  , tf_tolerance_(tf_tolerance)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_updated_ = node->now();
  temporal_tile_map_ = std::make_shared<SegmentationTileMap>(costmap_resolution, tile_map_decay_time);
  visualize_tile_map_ = visualize_tile_map;
  use_cost_selection_ = use_cost_selection;
  project_pointcloud_ = project_pointcloud;
  camera_optical_frame_ = camera_optical_frame;
  if (project_pointcloud_)
  {
    RCLCPP_INFO(logger_, "SegmentationBuffer [%s]: LiDAR-projection mode ON. Projection frame = %s",
                buffer_source_.c_str(),
                camera_optical_frame_.empty() ? "<CameraInfo frame_id> + physical->optical swap"
                                              : camera_optical_frame_.c_str());
  }
  RCLCPP_INFO(logger_, "SegmentationBuffer [%s]: Selection method = %s", 
              buffer_source_.c_str(), 
              use_cost_selection_ ? "COST-BASED (max_cost)" : "CONFIDENCE-BASED");
  if(visualize_tile_map_)
  {
    tile_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(buffer_source + "/tile_map",1);
  }
}

SegmentationBuffer::~SegmentationBuffer() {}

void SegmentationBuffer::createSegmentationCostMultimap(const vision_msgs::msg::LabelInfo& label_info)
{
  std::unordered_map<std::string, uint8_t> class_to_id_map;
  for (const auto& semantic_class : label_info.class_map)
  {
    const auto& name = semantic_class.class_name;
    if (class_names_cost_map_.find(name) == class_names_cost_map_.end()) {
      RCLCPP_ERROR(logger_, 
        "CRITICAL ERROR: Class '%s' from label_info is not defined in the costmap parameters! This class will be ignored.", 
        name.c_str());
      continue;
    }
    class_to_id_map[name] = semantic_class.class_id;
  }
  segmentation_cost_multimap_ = std::make_shared<SegmentationCostMultimap>(class_to_id_map, class_names_cost_map_);
}

void SegmentationBuffer::createSegmentationCostMultimapFromIds(
  const std::unordered_map<std::string, uint8_t>& class_name_to_id)
{
  segmentation_cost_multimap_ =
    std::make_shared<SegmentationCostMultimap>(class_name_to_id, class_names_cost_map_);
  RCLCPP_INFO(logger_,
              "SegmentationBuffer [%s]: class id map built from parameters (no LabelInfo topic).",
              buffer_source_.c_str());
}

void SegmentationBuffer::setValueRanges(const std::vector<SegmentationValueRange>& ranges)
{
  value_ranges_ = ranges;
  if (!value_ranges_.empty())
  {
    RCLCPP_INFO(logger_,
                "SegmentationBuffer [%s]: score-threshold mode ON (%zu value ranges); mask pixels "
                "are interpreted as a 0-255 score, not a class id.",
                buffer_source_.c_str(), value_ranges_.size());
  }
}

void SegmentationBuffer::setDirectCostMapping(double scale, double offset, int ignore_lo,
                                             int ignore_hi)
{
  direct_cost_ = true;
  direct_cost_scale_ = scale;
  direct_cost_offset_ = offset;
  direct_cost_ignore_lo_ = ignore_lo;
  direct_cost_ignore_hi_ = ignore_hi;

  // One "class" per possible cost (0..254), each mapping to itself, so the
  // existing tile / cost machinery works unchanged.
  std::unordered_map<std::string, uint8_t> name_to_id;
  std::unordered_map<std::string, CostHeuristicParams> name_to_cost;
  for (int i = 0; i <= 254; ++i)
  {
    const std::string name = "cost_" + std::to_string(i);
    name_to_id[name] = static_cast<uint8_t>(i);
    name_to_cost[name] =
      CostHeuristicParams{static_cast<uint8_t>(i), static_cast<uint8_t>(i), 0, 0, false};
  }
  segmentation_cost_multimap_ = std::make_shared<SegmentationCostMultimap>(name_to_id, name_to_cost);

  // Tile dominance = highest cost among non-decayed observations (worst / safest).
  temporal_tile_map_->setPreferMaxClassId(true);

  RCLCPP_INFO(logger_,
              "SegmentationBuffer [%s]: direct-cost mode ON (cost = clamp(%.1f*(1-v/255)+%.1f, "
              "0, 254)); no class binning.",
              buffer_source_.c_str(), direct_cost_scale_, direct_cost_offset_);
}

int SegmentationBuffer::resolveClassId(uint8_t pixel_value) const
{
  if (direct_cost_)
  {
    if (direct_cost_ignore_lo_ >= 0 && direct_cost_ignore_hi_ >= direct_cost_ignore_lo_ &&
        pixel_value >= direct_cost_ignore_lo_ && pixel_value <= direct_cost_ignore_hi_)
    {
      return -1;  // unknown band -> drop
    }
    double cost = direct_cost_scale_ * (1.0 - static_cast<double>(pixel_value) / 255.0) +
                  direct_cost_offset_;
    int c = static_cast<int>(std::lround(cost));
    return std::max(0, std::min(254, c));
  }
  if (value_ranges_.empty())
  {
    // Default: the pixel value is the class id (255 traversable / 0 obstacle...).
    return segmentation_cost_multimap_->hasClassId(pixel_value) ? static_cast<int>(pixel_value) : -1;
  }
  // Score mask: first range that contains the value wins.
  for (const auto& range : value_ranges_)
  {
    if (pixel_value >= range.lo && pixel_value <= range.hi)
    {
      return static_cast<int>(range.class_id);
    }
  }
  return -1;  // value falls in a gap (e.g. an "uncertain" band) -> ignore
}

void SegmentationBuffer::setCameraInfo(const sensor_msgs::msg::CameraInfo& info)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (!camera_projection_.fromCameraInfo(info))
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                         "SegmentationBuffer [%s]: received an unusable CameraInfo "
                         "(zero size / intrinsics). Ignoring.",
                         buffer_source_.c_str());
    return;
  }
  camera_info_frame_ = info.header.frame_id;
}

void SegmentationBuffer::bufferSegmentation(
  const sensor_msgs::msg::PointCloud2& cloud,
  const sensor_msgs::msg::Image& segmentation,
  const sensor_msgs::msg::Image& confidence)
{
  if (project_pointcloud_)
  {
    bufferProjectedSegmentation(cloud, segmentation, confidence);
    return;
  }

  geometry_msgs::msg::PointStamped global_origin;
  // check whether the origin frame has been set explicitly
  // or whether we should get it from the cloud
  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these segmentations come from sensors...
    // we'll need to store the origin pt of the sensor
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);

    sensor_msgs::msg::PointCloud2 global_frame_cloud;

    // transform the point cloud
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x_global(global_frame_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y_global(global_frame_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z_global(global_frame_cloud, "z");
    std::unordered_map<TileIndex, int> best_observations_idxs;
    double cloud_time_seconds = rclcpp::Time(cloud.header.stamp.sec, cloud.header.stamp.nanosec).seconds();

    // copy over the points that are within our segmentation range
    for (size_t v = 0; v < segmentation.height; v++)
    {
      for (size_t u = 0; u < segmentation.width; u++)
      {
        int pixel_idx = v * segmentation.width + u;
        // remove invalid points
        if (!std::isfinite(*(iter_z_global)))
        {
          ++iter_x_global;
          ++iter_y_global;
          ++iter_z_global;
          continue;
        }
        double sq_dist =
          std::pow(*(iter_x_global) - global_origin.point.x, 2) +
          std::pow(*(iter_y_global) - global_origin.point.y, 2) +
          std::pow(*(iter_z_global) - global_origin.point.z, 2);
        if (sq_dist >= sq_max_lookahead_distance_ || sq_dist <= sq_min_lookahead_distance_)
        {
          ++iter_x_global;
          ++iter_y_global;
          ++iter_z_global;
          continue;
        }

        TileIndex costmap_index = temporal_tile_map_->worldToIndex(*iter_x_global, *iter_y_global);

        // Selection policy per tile: cost-based (max_cost) or confidence-based
        auto it = best_observations_idxs.find(costmap_index);
        if (it != best_observations_idxs.end()) {
          if (use_cost_selection_) {
            // Cost-based: pick highest max_cost (of the resolved class).
            int current_class = resolveClassId(segmentation.data[pixel_idx]);
            int existing_class = resolveClassId(segmentation.data[it->second]);
            uint8_t current_max = current_class >= 0
              ? segmentation_cost_multimap_->getCostById(current_class).max_cost : 0;
            uint8_t existing_max = existing_class >= 0
              ? segmentation_cost_multimap_->getCostById(existing_class).max_cost : 0;
            if (current_max > existing_max) {
              best_observations_idxs[costmap_index] = pixel_idx;
              RCLCPP_DEBUG(logger_, "COST-BASED: Replaced tile observation - current_class=%d (max_cost=%d) > existing_class=%d (max_cost=%d)",
                          current_class, current_max, existing_class, existing_max);
            }
          } else {
            // Confidence-based: pick highest confidence
            if (confidence.data[pixel_idx] > confidence.data[it->second]) {
              best_observations_idxs[costmap_index] = pixel_idx;
              RCLCPP_DEBUG(logger_, "CONFIDENCE-BASED: Replaced tile observation - current_confidence=%d > existing_confidence=%d", 
                          confidence.data[pixel_idx], confidence.data[it->second]);
            }
          }
        } else {
          best_observations_idxs[costmap_index] = pixel_idx;
        }
        ++iter_x_global;
        ++iter_y_global;
        ++iter_z_global;
      }
    }

    commitObservations(best_observations_idxs, segmentation, confidence, cloud_time_seconds);

  } catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_,
                 "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
                 sensor_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();
}

void SegmentationBuffer::commitObservations(
  const std::unordered_map<TileIndex, int>& best_observations_idxs,
  const sensor_msgs::msg::Image& segmentation, const sensor_msgs::msg::Image& confidence,
  double cloud_time_seconds)
{
  temporal_tile_map_->lock();
  temporal_tile_map_->purgeOldObservations(cloud_time_seconds);
  for (const auto& idx : best_observations_idxs)
  {
    int img_idx_for_best_obs = idx.second;
    TileIndex costmap_index = idx.first;
    // Resolve the raw mask value to a class id: identity for a class-id mask
    // (255 traversable / 0 obstacle...), or range lookup for a score mask.
    int class_id = resolveClassId(segmentation.data[img_idx_for_best_obs]);

    if (class_id >= 0) {
      TileObservation best_obs{static_cast<uint8_t>(class_id),
                               static_cast<float>(confidence.data[img_idx_for_best_obs]),
                               cloud_time_seconds};
      bool dominant_priority = segmentation_cost_multimap_->getCostById(class_id).dominant_priority;
      temporal_tile_map_->pushObservation(best_obs, costmap_index, dominant_priority);
    } else {
      RCLCPP_DEBUG(logger_,
                   "SegmentationBuffer [%s]: Skipping unmapped mask value %d in tile (%d, %d)",
                   buffer_source_.c_str(), segmentation.data[img_idx_for_best_obs], costmap_index.x,
                   costmap_index.y);
    }
  }
  temporal_tile_map_->unlock();

  if (visualize_tile_map_)
  {
    sensor_msgs::msg::PointCloud2 tile_map_cloud = visualizeTemporalTileMap(*temporal_tile_map_);
    tile_map_pub_->publish(tile_map_cloud);
  }
}

void SegmentationBuffer::bufferProjectedSegmentation(
  const sensor_msgs::msg::PointCloud2& cloud, const sensor_msgs::msg::Image& segmentation,
  const sensor_msgs::msg::Image& confidence)
{
  if (!camera_projection_.valid())
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "SegmentationBuffer [%s]: no usable CameraInfo received yet, cannot "
                         "project LiDAR points. Skipping cloud.",
                         buffer_source_.c_str());
    return;
  }
  if (camera_projection_.width() != segmentation.width ||
      camera_projection_.height() != segmentation.height)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "SegmentationBuffer [%s]: CameraInfo size (%ux%u) != segmentation image "
                         "size (%ux%u). Skipping cloud.",
                         buffer_source_.c_str(), camera_projection_.width(),
                         camera_projection_.height(), segmentation.width, segmentation.height);
    return;
  }

  // Frame the LiDAR points must be projected from. When an explicit optical
  // frame is configured we let TF put the points straight into it. Otherwise we
  // transform into the physical camera frame advertised by CameraInfo
  // (e.g. "rgb_camera_frame") and apply the repo's physical->optical axis
  // convention by hand, because that frame is NOT a REP-103 optical frame and
  // this robot's TF tree has no optical frame at all.
  const bool have_optical_frame = !camera_optical_frame_.empty();
  const std::string projection_frame =
    have_optical_frame ? camera_optical_frame_ : camera_info_frame_;
  if (projection_frame.empty())
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "SegmentationBuffer [%s]: no projection frame known yet. Skipping cloud.",
                         buffer_source_.c_str());
    return;
  }

  const int width = static_cast<int>(segmentation.width);
  const int height = static_cast<int>(segmentation.height);
  const double cloud_time_seconds =
    rclcpp::Time(cloud.header.stamp.sec, cloud.header.stamp.nanosec).seconds();

  try
  {
    // Sensor origin in the global frame, for the range gate (same semantics as
    // the aligned path).
    geometry_msgs::msg::PointStamped local_origin, global_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = cloud.header.frame_id;
    local_origin.point.x = local_origin.point.y = local_origin.point.z = 0.0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);

    // Two views of the same cloud: one for tile binning, one for projection.
    // tf2 preserves point order, so both can be iterated in lockstep.
    sensor_msgs::msg::PointCloud2 global_cloud, camera_cloud;
    tf2_buffer_.transform(cloud, global_cloud, global_frame_, tf_tolerance_);
    tf2_buffer_.transform(cloud, camera_cloud, projection_frame, tf_tolerance_);

    sensor_msgs::PointCloud2ConstIterator<float> gx(global_cloud, "x"), gy(global_cloud, "y"),
      gz(global_cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> cx(camera_cloud, "x"), cy(camera_cloud, "y"),
      cz(camera_cloud, "z");

    std::unordered_map<TileIndex, int> best_observations_idxs;
    const size_t n_points = static_cast<size_t>(camera_cloud.width) * camera_cloud.height;

    for (size_t i = 0; i < n_points; ++i, ++gx, ++gy, ++gz, ++cx, ++cy, ++cz)
    {
      // Drop invalid / NaN / Inf points in either representation.
      if (!std::isfinite(*cx) || !std::isfinite(*cy) || !std::isfinite(*cz) ||
          !std::isfinite(*gx) || !std::isfinite(*gy) || !std::isfinite(*gz))
      {
        continue;
      }

      // Into the optical convention expected by CameraProjection::project().
      double ox, oy, oz;
      if (have_optical_frame)
      {
        ox = *cx;
        oy = *cy;
        oz = *cz;
      }
      else
      {
        CameraProjection::opticalFromPhysical(*cx, *cy, *cz, ox, oy, oz);
      }

      // Project. Returns false for points on/behind the image plane.
      double u_d, v_d;
      if (!camera_projection_.project(ox, oy, oz, u_d, v_d))
      {
        continue;
      }
      const int u = static_cast<int>(std::lround(u_d));
      const int v = static_cast<int>(std::lround(v_d));
      if (u < 0 || v < 0 || u >= width || v >= height)
      {
        continue;
      }

      // Range gate in the global frame.
      const double sq_dist = std::pow(*gx - global_origin.point.x, 2) +
                             std::pow(*gy - global_origin.point.y, 2) +
                             std::pow(*gz - global_origin.point.z, 2);
      if (sq_dist >= sq_max_lookahead_distance_ || sq_dist <= sq_min_lookahead_distance_)
      {
        continue;
      }

      const int pixel_idx = v * width + u;
      const TileIndex costmap_index = temporal_tile_map_->worldToIndex(*gx, *gy);

      // One observation per tile per frame. Many LiDAR points can land on the
      // same pixel / tile; keep one using the same policy as the aligned path.
      auto it = best_observations_idxs.find(costmap_index);
      if (it == best_observations_idxs.end())
      {
        best_observations_idxs[costmap_index] = pixel_idx;
        continue;
      }
      if (use_cost_selection_)
      {
        int current_class = resolveClassId(segmentation.data[pixel_idx]);
        int existing_class = resolveClassId(segmentation.data[it->second]);
        uint8_t current_max = current_class >= 0
          ? segmentation_cost_multimap_->getCostById(current_class).max_cost : 0;
        uint8_t existing_max = existing_class >= 0
          ? segmentation_cost_multimap_->getCostById(existing_class).max_cost : 0;
        if (current_max > existing_max)
        {
          it->second = pixel_idx;
        }
      }
      else if (confidence.data[pixel_idx] > confidence.data[it->second])
      {
        it->second = pixel_idx;
      }
    }

    commitObservations(best_observations_idxs, segmentation, confidence, cloud_time_seconds);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "SegmentationBuffer [%s]: TF error projecting LiDAR (%s -> %s / %s): %s",
                         buffer_source_.c_str(), cloud.header.frame_id.c_str(),
                         global_frame_.c_str(), projection_frame.c_str(), ex.what());
    return;
  }

  last_updated_ = clock_->now();
}


std::unordered_map<std::string, CostHeuristicParams> SegmentationBuffer::getClassMap()
{
  return class_names_cost_map_;
}

std::vector<std::string> SegmentationBuffer::getClassNamesForType(const std::string& class_type)
{
  auto it = class_type_to_names_.find(class_type);
  if (it != class_type_to_names_.end())
  {
    return it->second;
  }
  return std::vector<std::string>();
}


void SegmentationBuffer::updateClassMap(std::string new_class, CostHeuristicParams new_cost)
{
  segmentation_cost_multimap_->updateCostByName(new_class, new_cost);
}

bool SegmentationBuffer::isCurrent() const
{
  if (expected_update_rate_ == rclcpp::Duration(0.0s))
  {
    return true;
  }

  bool current = (clock_->now() - last_updated_) <= expected_update_rate_;
  if (!current)
  {
    RCLCPP_WARN(logger_,
                "The %s segmentation buffer has not been updated for %.2f seconds, "
                "and it should be updated every %.2f seconds.",
                buffer_source_.c_str(), (clock_->now() - last_updated_).seconds(),
                expected_update_rate_.seconds());
  }
  return current;
}

void SegmentationBuffer::resetLastUpdated() { last_updated_ = clock_->now(); }
}  // namespace semantic_segmentation_layer
