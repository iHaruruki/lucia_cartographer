include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2, --> Subscribe するレーザースキャントピックの数
  num_multi_echo_laser_scans = 0, --> Subscribe するマルチエコーレーザースキャンのトピック数
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.05,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4 --> バックグラウンドでの計算に使用するスレッド数

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.huber_scale = 1e3
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4 --> ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5 --> ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.matcher_translation_weight = 5e2 --> 非ループクロージャー制約の"並進"成分に対する最適化問題で使用される重み
POSE_GRAPH.matcher_rotation_weight = 1.6e3 --> 非ループクロージャー制約の"回転"成分に対する最適化問題で使用される重み

TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

return options
