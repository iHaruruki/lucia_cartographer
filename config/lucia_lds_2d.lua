include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- LiDAR が1台なら 1、2台なら 2（Remap 必須）
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4

-- ループ閉じ検出を強める
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0
POSE_GRAPH.constraint_builder.min_score = 0.52
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.56
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 12.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(40.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.log_matches = true

-- マッチャの重み
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- Odometry あり想定。強すぎるとループ効果が出ないため 1e2 から
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight   = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight       = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight          = 1e2
POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- 2D ビルダー設定
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- ローカル相関（必要時のみ有効化）
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

return options