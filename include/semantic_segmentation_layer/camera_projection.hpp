// Copyright 2026 robot.com
//
// Software License Agreement (Apache-2.0)
//
// Authors: Pedro Gonzalez (pedro@robot.com)
//          Johan Solarte (jsolarte@robot.com)

#ifndef SEMANTIC_SEGMENTATION_LAYER__CAMERA_PROJECTION_HPP_
#define SEMANTIC_SEGMENTATION_LAYER__CAMERA_PROJECTION_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace semantic_segmentation_layer
{

/**
 * @brief Pinhole projection helper built from a sensor_msgs/CameraInfo.
 *
 * Frame conventions
 * -----------------
 * project() expects a point already expressed in the camera OPTICAL frame
 * (REP 103): +x right, +y down, +z forward (into the scene).
 *
 * The physical camera frame advertised by CameraInfo.header.frame_id
 * (e.g. "rgb_camera_frame") is NOT assumed to follow that convention. When a
 * dedicated optical frame exists in TF, transform the points straight into it
 * and call project() directly. When it does not (as in this robot's TF tree,
 * which only has base_link -> rgb_camera_frame), opticalFromPhysical() converts
 * a point from the repo's physical-camera convention (+x forward, +y left,
 * +z up, see zeta_prompt/project_path_to_image.py) into the optical convention.
 */
class CameraProjection
{
public:
  /**
   * @brief Load intrinsics from a CameraInfo message.
   *
   * Uses K + D (so the projected pixel matches the raw, distorted image the
   * segmentation mask is produced from). Falls back to P only when K is unset.
   * @return false if the message carries no usable size / intrinsics.
   */
  bool fromCameraInfo(const sensor_msgs::msg::CameraInfo & info)
  {
    if (info.width == 0 || info.height == 0) {
      return false;
    }
    if (info.k[0] != 0.0) {
      // K is the 3x3 row-major intrinsic matrix: [fx 0 cx; 0 fy cy; 0 0 1].
      fx_ = info.k[0];
      fy_ = info.k[4];
      cx_ = info.k[2];
      cy_ = info.k[5];
      distortion_ = info.d;
      distortion_model_ = info.distortion_model;
    } else if (info.p[0] != 0.0) {
      // P is the 3x4 row-major projection matrix (rectified image, distortion
      // already removed): [fx 0 cx Tx; 0 fy cy Ty; 0 0 1 0].
      fx_ = info.p[0];
      fy_ = info.p[5];
      cx_ = info.p[2];
      cy_ = info.p[6];
      distortion_.clear();
      distortion_model_.clear();
    } else {
      return false;
    }
    if (fx_ == 0.0 || fy_ == 0.0) {
      return false;
    }
    width_ = info.width;
    height_ = info.height;
    valid_ = true;
    return true;
  }

  bool valid() const {return valid_;}
  std::uint32_t width() const {return width_;}
  std::uint32_t height() const {return height_;}

  /**
   * @brief Project an OPTICAL-frame point to pixel coordinates.
   * @param min_z points with z <= min_z are on/behind the image plane.
   * @return false when the point is on/behind the image plane (not projectable).
   */
  bool project(double x, double y, double z, double & u, double & v, double min_z = 1e-3) const
  {
    if (!(z > min_z)) {  // also rejects NaN
      return false;
    }
    double xn = x / z;
    double yn = y / z;
    if (hasDistortion()) {
      applyRadTan(xn, yn);
    }
    u = fx_ * xn + cx_;
    v = fy_ * yn + cy_;
    return std::isfinite(u) && std::isfinite(v);
  }

  /**
   * @brief Repo physical-camera convention (+x fwd, +y left, +z up) -> optical
   *        convention (+x right, +y down, +z fwd).
   *
   * Matches zeta_prompt/project_path_to_image.py. Only used when TF has no
   * dedicated optical frame; prefer a real optical frame when one exists.
   */
  static void opticalFromPhysical(
    double px, double py, double pz, double & ox, double & oy, double & oz)
  {
    ox = -py;
    oy = -pz;
    oz = px;
  }

private:
  bool hasDistortion() const
  {
    if (distortion_.size() < 5) {
      return false;
    }
    for (double c : distortion_) {
      if (std::abs(c) > 1e-9) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief plumb_bob / rational_polynomial radial-tangential distortion.
   *        D = [k1, k2, p1, p2, k3, (k4, k5, k6)].
   */
  void applyRadTan(double & x, double & y) const
  {
    const double k1 = distortion_[0], k2 = distortion_[1];
    const double p1 = distortion_[2], p2 = distortion_[3], k3 = distortion_[4];
    const double k4 = distortion_.size() > 5 ? distortion_[5] : 0.0;
    const double k5 = distortion_.size() > 6 ? distortion_[6] : 0.0;
    const double k6 = distortion_.size() > 7 ? distortion_[7] : 0.0;
    const double r2 = x * x + y * y;
    const double radial =
      (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1.0 + ((k6 * r2 + k5) * r2 + k4) * r2);
    const double x_tan = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double y_tan = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    x = x * radial + x_tan;
    y = y * radial + y_tan;
  }

  bool valid_ = false;
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  std::uint32_t width_ = 0, height_ = 0;
  std::string distortion_model_;
  std::vector<double> distortion_;
};

}  // namespace semantic_segmentation_layer

#endif  // SEMANTIC_SEGMENTATION_LAYER__CAMERA_PROJECTION_HPP_
