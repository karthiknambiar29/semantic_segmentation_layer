![robot.com banner](docs/banner.png)

# Semantic Segmentation Layer

A [Nav2](https://nav2.org) costmap layer plugin that turns semantic segmentation into navigation cost. Inspired by the [Spatio-Temporal Voxel Layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer), it accumulates per-class observations on costmap tiles over time and lets them decay, so the costmap reflects what the robot has seen recently rather than a single noisy frame.

Originally developed by [@pepisg](https://github.com/pepisg) and [@sunart24](https://github.com/sunart24) at [robot.com](https://robot.com). This fork adds a **LiDAR-projection mode**, **score-threshold classes** and a **direct-cost mode** for continuous traversability masks.

![video](docs/video.gif)

## Contents

- [Features](#features)
- [Input modes](#input-modes)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Using the layer in Nav2](#using-the-layer-in-nav2)
- [Topics](#topics)
- [Parameters](#parameters)
- [Mask interpretation](#mask-interpretation)
- [How it works](#how-it-works)
- [Frames and TF](#frames-and-tf)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Package layout](#package-layout)
- [Credits, citation and license](#credits)

## Features

- Two geometry sources per observation source:
  - **RGBD mode**: a pixel-aligned, organized point cloud (one 3D point per mask pixel).
  - **LiDAR-projection mode**: an unorganized LiDAR cloud projected into the mask using `sensor_msgs/CameraInfo` intrinsics (pinhole + `plumb_bob` / `rational_polynomial` distortion).
- Three ways to read the mask:
  - **Class IDs**: each pixel value is a class ID, named via `vision_msgs/LabelInfo` or the `class_ids` parameter.
  - **Score thresholds**: bin a continuous 0-255 score into named classes with `value_min` / `value_max`.
  - **Direct cost**: map the pixel value straight to a costmap cost, no classes needed.
- Optional per-pixel confidence image.
- Per-class cost heuristics: `base_cost`, `max_cost`, `mark_confidence`, `samples_to_max_cost`, `dominant_priority`.
- Temporal decay of observations (`tile_map_decay_time`).
- Exact or approximate time synchronization of inputs.
- Multiple observation sources (cameras) in one layer.
- Optional tile-map visualization as a `PointCloud2`.
- Standalone costmap launch file and a synthetic input publisher for a robot-free smoke test.

## Input modes

| | RGBD mode (`project_pointcloud: false`) | LiDAR-projection mode (`project_pointcloud: true`) |
|---|---|---|
| Point cloud | Organized, `width*height` equal to the mask | Any unorganized cloud (`x`, `y`, `z` float32 fields) |
| Pixel lookup | Point index = pixel index | Transform to camera frame, project with intrinsics |
| `camera_info_topic` | Not used | **Required** |
| Time sync | Exact by default (`approximate_sync` optional) | Always approximate (forced on) |
| Typical sensor | Depth camera with aligned depth | Separate LiDAR + RGB camera |

## Dependencies

Tested on **ROS 2 Humble** (Ubuntu 22.04). The CMake uses `ament_target_dependencies`, so it also builds on newer distros that still provide it.

Build tool: `ament_cmake`, `nav2_common`

Runtime / build:

| Package | Used for |
|---|---|
| `nav2_costmap_2d`, `nav2_util` | Costmap layer base class, lifecycle node |
| `pluginlib` | Plugin export |
| `rclcpp`, `rclcpp_lifecycle` | ROS 2 client library |
| `message_filters` | Exact / approximate time synchronizers |
| `sensor_msgs` | `Image`, `PointCloud2`, `CameraInfo` |
| `vision_msgs` | `LabelInfo` (class name to ID map) |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_sensor_msgs` | Transforming clouds and sensor origins |
| `geometry_msgs`, `nav_msgs`, `std_msgs` | Messages |

Launch file / smoke test (not declared in `package.xml`, install if you use them): `nav2_lifecycle_manager`, `launch`, `launch_ros`, `rclpy`, `tf2_ros` (Python).

Tests: `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`.

## Installation

```bash
# 1. ROS 2 and Nav2
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-vision-msgs \
                 ros-humble-tf2-sensor-msgs python3-colcon-common-extensions python3-rosdep

# 2. Get the source into a workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/karthiknambiar29/semantic_segmentation_layer.git

# 3. Remaining dependencies
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y

# 4. Build
colcon build --packages-select semantic_segmentation_layer --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Check that pluginlib can see the plugin:

```bash
ros2 pkg prefix semantic_segmentation_layer
grep -r SemanticSegmentationLayer $(ros2 pkg prefix semantic_segmentation_layer)/share/semantic_segmentation_layer/costmap_plugins.xml
```

## Quick start

The package ships a standalone costmap (`nav2_costmap_2d` node + lifecycle manager) using
[`config/segmentation_layer_local_costmap.yaml`](config/segmentation_layer_local_costmap.yaml)
(LiDAR-projection + direct-cost mode).

**Smoke test, no robot needed.** Publishes synthetic TF, a LiDAR grid, a gradient mask and CameraInfo:

```bash
ros2 launch semantic_segmentation_layer segmentation_layer_costmap.launch.py publish_fake_inputs:=true
```

**Against a real robot or bag** (TF and sensors already running):

```bash
ros2 launch semantic_segmentation_layer segmentation_layer_costmap.launch.py \
    params_file:=/path/to/my_params.yaml use_sim_time:=true
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `params_file` | `share/.../config/segmentation_layer_local_costmap.yaml` | Costmap parameters |
| `use_sim_time` | `false` | Use `/clock` |
| `publish_test_tf` | `false` | Identity static TFs `odom -> base_link -> {rgb_camera_frame, lidar_frame}` (no sensor data) |
| `publish_fake_inputs` | `false` | Run `test/fake_inputs.py` (TF + LiDAR + mask + CameraInfo) |

Inspect the result:

```bash
ros2 topic hz /costmap/costmap
rviz2   # add Map display on /costmap/costmap, fixed frame: odom
```

The costmap only publishes once TF `global_frame -> robot_base_frame` and sensor data are available.

## Using the layer in Nav2

Add the plugin to `local_costmap` and/or `global_costmap` in your Nav2 params file:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["segmentation_layer", "inflation_layer"]
      segmentation_layer:
        plugin: "semantic_segmentation_layer::SemanticSegmentationLayer"
        enabled: true
        combination_method: 1        # 0 overwrite, 1 max
        observation_sources: camera  # space-separated list
        camera:
          segmentation_topic: "/segmentation/mask"
          confidence_topic: "/segmentation/confidence"   # "" to disable
          labels_topic: "/segmentation/label_info"       # "" to use class_ids
          pointcloud_topic: "/rgbd_camera/depth/points"
          max_obstacle_distance: 5.0
          min_obstacle_distance: 0.3
          tile_map_decay_time: 5.0
          use_cost_selection: false
          class_types: ["traversable", "danger"]
          traversable:
            classes: ["sidewalk"]
            base_cost: 0
            max_cost: 0
          danger:
            classes: ["grass"]
            base_cost: 254
            max_cost: 254
```

Place the layer before `inflation_layer` so its costs get inflated. For a full example with simulated models, see [nav2_segmentation_demo](https://github.com/pepisg/nav2_segmentation_demo).

## Topics

### Subscribed (per observation source)

| Parameter | Type | QoS | Notes |
|---|---|---|---|
| `segmentation_topic` | `sensor_msgs/Image` (mono8) | sensor data, depth 50 | Class ID or score per pixel |
| `pointcloud_topic` | `sensor_msgs/PointCloud2` | sensor data, depth 50 | RGBD: pixel-aligned. LiDAR mode: any cloud. Passed through a `tf2_ros::MessageFilter` to `global_frame` |
| `confidence_topic` (optional) | `sensor_msgs/Image` (mono8) | sensor data, depth 50 | 0-255 per pixel, same size as mask. When empty, confidence is 255 everywhere |
| `labels_topic` (optional) | `vision_msgs/LabelInfo` | reliable, transient local, depth 5 | Class name to ID. Nothing is buffered until one arrives |
| `camera_info_topic` (LiDAR mode) | `sensor_msgs/CameraInfo` | default, depth 5 | Latest message cached. Size must match the mask |

Inputs are synchronized: `segmentation + pointcloud`, or `segmentation + confidence + pointcloud`.

### Published

| Topic | Type | When |
|---|---|---|
| `<costmap node>/<source>/tile_map` | `sensor_msgs/PointCloud2` | `visualize_tile_map: true`. One point per tile with fields `x y z confidence confidence_sum class` |

## Parameters

All parameters are under `<layer_name>.`.

### Layer

| Name | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Enable the layer (dynamic) |
| `combination_method` | int | `1` | `0` overwrite master grid, `1` max, other: no-op |
| `observation_sources` | string | `""` | Space-separated source names |

Also read from the costmap: `track_unknown_space`, `transform_tolerance`.

### Per source (`<layer_name>.<source>.`)

| Name | Type | Default | Description |
|---|---|---|---|
| `segmentation_topic` | string | `""` | Mask topic |
| `confidence_topic` | string | `""` | Confidence topic, empty disables |
| `labels_topic` | string | `""` | `LabelInfo` topic, empty requires `class_ids` (unless `direct_cost`) |
| `pointcloud_topic` | string | `""` | Point cloud topic |
| `max_obstacle_distance` | double | `5.0` | Points at or beyond this 3D distance from the cloud origin are ignored (dynamic) |
| `min_obstacle_distance` | double | `0.3` | Points at or within this distance are ignored (dynamic) |
| `tile_map_decay_time` | double | `5.0` | Seconds an observation stays in a tile queue |
| `use_cost_selection` | bool | `true` | Several pixels on one tile in one frame: keep highest `max_cost` (`true`) or highest confidence (`false`) |
| `visualize_tile_map` | bool | `false` | Publish `<source>/tile_map` |
| `class_types` | string[] | `[]` | Names of class groups (required unless `direct_cost`) |
| `observation_persistence` | double | `0.0` | Stored in the buffer, currently unused |
| `expected_update_rate` | double | `0.0` | Stored in the buffer, currently unused |
| `project_pointcloud` | bool | `false` | Enable LiDAR-projection mode |
| `camera_info_topic` | string | `""` | Required when `project_pointcloud` is true |
| `camera_optical_frame` | string | `""` | REP-103 optical frame to project from. Empty: use `CameraInfo.header.frame_id` with a physical-to-optical axis swap (see [Frames and TF](#frames-and-tf)) |
| `approximate_sync` | bool | `false` | Use `ApproximateTime` sync (forced `true` in LiDAR mode) |
| `approximate_sync_tolerance` | double | `0.05` | Max stamp difference (s) between synced messages |
| `direct_cost` | bool | `false` | Map pixel value directly to cost |
| `direct_cost_scale` | double | `254.0` | See formula below |
| `direct_cost_offset` | double | `0.0` | See formula below |
| `direct_cost_ignore_min` | int | `-1` | Drop pixels with value in `[ignore_min, ignore_max]`; `-1` disables |
| `direct_cost_ignore_max` | int | `-1` | |

### Per class type (`<layer_name>.<source>.<class_type>.`)

| Name | Type | Default | Description |
|---|---|---|---|
| `classes` | string[] | `[]` | Class names in this group (must match `LabelInfo` names if used) |
| `class_ids` | int[] | `[]` | IDs for each entry in `classes`, same order. Required when `labels_topic` is empty |
| `value_min`, `value_max` | int | `-1` | Score range 0-255 mapped to `class_ids[0]`. Both `-1` disables |
| `base_cost` | int | `0` | Cost when the tile has fewer than `samples_to_max_cost` observations or low confidence (dynamic) |
| `max_cost` | int | `0` | Cost when the tile has `>= samples_to_max_cost` observations **and** average confidence `> mark_confidence` (dynamic) |
| `mark_confidence` | int | `0` | Average confidence threshold, 0-255 (dynamic) |
| `samples_to_max_cost` | int | `0` | Observations needed for `max_cost` (dynamic) |
| `dominant_priority` | bool | `false` | A new observation of this class takes over the tile immediately and clears other classes |

Note: with the defaults `samples_to_max_cost: 0` and `mark_confidence: 0`, any tile with non-zero confidence gets `max_cost`. Set both to use `base_cost` as a "seen once" cost.

Costs follow Nav2 conventions: `0` free, `253` inscribed, `254` lethal.

## Mask interpretation

The layer resolves each pixel value to a class ID (or cost) in this order.

### 1. Direct cost (`direct_cost: true`)

```
cost = clamp(round(direct_cost_scale * (1 - value/255) + direct_cost_offset), 0, 254)
```

With defaults, `255 -> 0` (free) and `0 -> 254` (lethal). Use a negative scale plus an offset to invert. Pixels in the ignore band are dropped. `class_types`, `labels_topic` and `class_ids` are ignored, `use_cost_selection` is forced on, and each tile reports the **highest** cost among its non-decayed observations (lower-cost observations stay as a fallback once it decays).

```yaml
direct_cost: true
direct_cost_scale: 254.0
direct_cost_offset: 0.0
direct_cost_ignore_min: 120   # treat 120..140 as "unknown"
direct_cost_ignore_max: 140
```

### 2. Score thresholds (`value_min` / `value_max`)

For continuous masks. The first range containing the value wins; values in gaps are ignored.

```yaml
use_cost_selection: true
class_types: ["obstacle", "traversable"]
obstacle:
  classes: ["obstacle"]
  class_ids: [1]
  value_min: 0
  value_max: 130
  base_cost: 254
  max_cost: 254
  dominant_priority: true
traversable:
  classes: ["traversable"]
  class_ids: [2]
  value_min: 146
  value_max: 255
  base_cost: 0
  max_cost: 0
```

If you also set `labels_topic`, the ranges still map to `class_ids`, so they must match the `LabelInfo` IDs.

### 3. Class IDs (default)

The pixel value is the class ID. Names come from `LabelInfo` or, if `labels_topic` is empty, from `class_ids` (one per class name, or the node exits). Unknown IDs are ignored.

```yaml
labels_topic: ""
class_types: ["traversable", "obstacle"]
traversable: {classes: ["traversable"], class_ids: [255], base_cost: 0}
obstacle:    {classes: ["obstacle"],    class_ids: [0],   base_cost: 254, dominant_priority: true}
```

## How it works

### Observations and queues

An **observation** is one class reading on one tile at one time: class ID, confidence (0-255) and the cloud timestamp. Each costmap tile keeps a **queue per class** observed there, ordered by time, with a running confidence sum. Observations older than `tile_map_decay_time` are purged.

Tiles are indexed in `global_frame` at the costmap resolution, independently of the costmap window, so observations survive a rolling window moving.

### Pipeline

1. **Sync**: mask, cloud (and confidence) arrive together. In RGBD mode, mismatched sizes are dropped.
2. **Pixel to tile**:
   - RGBD: point `i` belongs to pixel `i`.
   - LiDAR: each point is transformed to `global_frame` (for binning) and to the camera frame (for projection), projected with the intrinsics, and dropped if behind the camera or outside the image.
   - Points that are non-finite or outside `[min, max]_obstacle_distance` from the cloud origin are dropped.
3. **One observation per tile per frame**: highest `max_cost` (`use_cost_selection: true`) or highest confidence.
4. **Push and dominance**: the observation goes into its class queue. The tile's **dominant class** decides its cost:
   - `dominant_priority: true` classes take over immediately and clear the other queues.
   - Otherwise a class becomes dominant only when its queue is longer than the current dominant queue.
   - Direct-cost mode: highest cost wins.
5. **Cost** (in `updateBounds`): after purging old observations, the tile gets `max_cost` if `size >= samples_to_max_cost` and `confidence_sum/size > mark_confidence`, else `base_cost`. If the dominant queue decays empty, dominance is recomputed from the remaining queues.
6. **Merge** into the master grid with `combination_method`.

### Visual explanation

Purple and green are different classes; darker means higher confidence. Each stack on a tile is a class queue, the tallest/darkest one is dominant.

![Semantic Segmentation Layer Workflow](docs/segmentation_layer_workflow.png)

- **t=0**: One observation per tile, each marked with its class `base_cost`.
- **t=1**:
  - Left (purple): enough observations above `mark_confidence`, now `max_cost`.
  - Middle: two classes on one tile in the same frame; the higher `max_cost` or confidence wins (blue).
  - Right (green): more observations, still `base_cost`.
- **t=2**:
  - Left: no new observation, queue persists at `max_cost`.
  - Middle: purple with `dominant_priority=True` arrives, clears blue and takes over.
  - Right: blue arrives in a new queue, green stays dominant (longer).
- **t=3**:
  - Left: oldest observation decays, back to `base_cost`.
  - Middle: green arrives, purple stays dominant.
  - Right: blue ties green, green stays dominant (ties keep the incumbent).
- **t=4**:
  - Left: another observation decays.
  - Middle: green is now longer than purple, becomes dominant and purges purple.
  - Right: blue (3) beats green (2) and becomes dominant; `max_cost` if its average confidence passes `mark_confidence`.

## Frames and TF

Required TF: `global_frame -> robot_base_frame` for the costmap, and `global_frame -> <cloud frame>` at the cloud stamp (within `transform_tolerance`).

LiDAR-projection mode additionally needs `<cloud frame> -> <projection frame>`:

- **`camera_optical_frame` set**: points are transformed into that frame and projected directly. It must follow REP-103 optical convention (+x right, +y down, +z forward).
- **`camera_optical_frame` empty**: points are transformed into `CameraInfo.header.frame_id`, assumed to be a **physical** camera frame (+x forward, +y left, +z up), then swapped:
  `x_opt = -y`, `y_opt = -z`, `z_opt = x`.

If your `CameraInfo` frame is already an optical frame, set `camera_optical_frame` to it, otherwise the swap is applied twice and nothing projects.

Intrinsics use `K` and `D` (distorted image) when `K` is set, else `P` with no distortion.

## Testing

```bash
cd ~/ros2_ws
colcon build --packages-select semantic_segmentation_layer
colcon test --packages-select semantic_segmentation_layer --ctest-args -R test_
colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `test_camera_projection` | CameraInfo loading, pinhole projection, behind-camera / NaN rejection, physical-vs-optical frame, radial distortion |
| `test_projection_buffer` | LiDAR-projection buffering end to end: class lookup, out-of-image rejection, one observation per tile, TF rotation, missing TF / CameraInfo, score thresholds, direct cost, ignore band |

Without `-R test_`, `colcon test` also runs `ament_lint_common` linters, which may report style issues.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Class map is empty because a labelinfo message has not been received` | `labels_topic` set but no `LabelInfo` published (it must be transient local / latched), or set `labels_topic: ""` and use `class_ids` |
| `CRITICAL ERROR: Class 'X' from label_info is not defined` | A `LabelInfo` class name is missing from every `classes` list |
| Node exits: `no labels_topic and an incomplete class_ids` | One `class_ids` entry is needed per name in `classes` |
| Node exits: `project_pointcloud=true but no camera_info_topic` | Set `camera_info_topic` |
| `Pointcloud and segmentation sizes are different` | RGBD mode needs an organized cloud with the same `width*height` as the mask. Use LiDAR mode for unorganized clouds |
| `no usable CameraInfo received yet` | CameraInfo not published, or zero size / intrinsics |
| `CameraInfo size != segmentation image size` | Mask was resized; publish a CameraInfo matching the mask resolution |
| `TF error projecting LiDAR` | Missing transform, or stamps outside the TF buffer (check `use_sim_time`) |
| Nothing is synced / costmap stays empty | Stamps differ: enable `approximate_sync` and increase `approximate_sync_tolerance`; check that publishers use best-effort compatible QoS |
| LiDAR points land in the wrong place in the image | Wrong frame convention; see [Frames and TF](#frames-and-tf) |
| Everything marked `max_cost` | `samples_to_max_cost` / `mark_confidence` left at 0 |
| Obstacles linger | Lower `tile_map_decay_time` |

Enable `visualize_tile_map: true` and view `<source>/tile_map` in RViz to see what the layer is buffering.

## Package layout

```
semantic_segmentation_layer/
├── CMakeLists.txt
├── package.xml
├── costmap_plugins.xml                      # pluginlib description
├── config/segmentation_layer_local_costmap.yaml
├── launch/segmentation_layer_costmap.launch.py
├── include/semantic_segmentation_layer/
│   ├── semantic_segmentation_layer.hpp      # Layer plugin
│   ├── segmentation_buffer.hpp              # Buffer, tile map, queues, cost multimap
│   └── camera_projection.hpp                # Pinhole + distortion from CameraInfo
├── src/
│   ├── semantic_segmentation_layer.cpp      # Params, subscriptions, sync, updateBounds/Costs
│   └── segmentation_buffer.cpp              # RGBD and LiDAR-projection buffering
├── test/
│   ├── test_camera_projection.cpp
│   ├── test_projection_buffer.cpp
│   └── fake_inputs.py                       # Synthetic TF + LiDAR + mask + CameraInfo
└── docs/                                    # Images
```

## Known limitations

- `dominant_priority` is declared but not applied when changed at runtime (only the integer cost parameters and distances are dynamic).
- `observation_persistence`, `expected_update_rate` and `publish_debug_topics` are declared but have no effect.
- Invalid configuration calls `exit(-1)` during `onInitialize` instead of failing the lifecycle transition.
- One observation per tile per frame: tall objects and the ground under them compete for the same tile.

<a id="credits"></a>
## Credits, citation and license

### Authors

- **[@pepisg](https://github.com/pepisg)** - <pepisg@robot.com>
- **[@sunart24](https://github.com/sunart24)** - <sunart24@robot.com>

### Motivation

Traversability is often not only geometric: grass, sidewalk, mud or road surfaces matter as much as obstacles. This plugin brings that semantic information into Nav2.

### Acknowledgments

Inspired by the [Spatio-Temporal Voxel Layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer), extending its temporal buffering to semantic segmentation with multi-class queues.

### Citation

```bibtex
@software{semantic_segmentation_layer,
  author = {Gonzale, Pedro and Solarte, Johan},
  title = {Semantic Segmentation Layer: A Nav2 Costmap Plugin for RGBD Semantic Segmentation},
  year = {2026},
  url = {https://github.com/kiwicampus/semantic_segmentation_layer}
}
```

### Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes
4. Push and open a Pull Request

### License

Apache License 2.0. See [LICENSE](LICENSE).
