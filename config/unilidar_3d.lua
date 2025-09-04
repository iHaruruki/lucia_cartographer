-- Cartographer 3D configuration
-- 3D マップ用の設定例です。3D LiDAR（PointCloud2）と IMU が必要です。
-- 使用するフレーム名やトピックは環境に合わせて調整してください。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- フレーム設定
  map_frame = "map",
  -- 可能なら IMU に剛体固定されたフレーム（例: "imu_link"）を推奨
  tracking_frame = "unilidar_imu",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,                -- 既存のodomを使用する場合はfalse
  publish_frame_projected_to_2d = false,     -- 3Dなのでfalseのまま

  -- センサ利用
  use_odometry = false,       -- ホイールオドメトリがあるならtrue
  use_nav_sat = false,
  use_landmarks = false,

  -- 3Dではレーザスキャンは使わず、ポイントクラウドを使用
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,      -- 3D LiDARが1台なら1

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

-- 3Dを有効化
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

-- ループ閉じ検出（やや強め）
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.log_matches = true

-- マッチャの重み
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- Odometry あり想定。強すぎるとループ効果が出ないため 1e2 から
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight   = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight       = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight          = 1e2

-- 3D ビルダー設定（代表的な値）
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- 入力点群のボクセルダウンサンプリング
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- 高解像・低解像の適応ボクセルフィルタ
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 20.

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.

-- ループ用の適応ボクセルフィルタ
TRAJECTORY_BUILDER_3D.loop_closure_adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_3D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.loop_closure_adaptive_voxel_filter.max_range = 30.

-- Ceres スキャンマッチャ
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

-- サブマップ解像度と蓄積点群数
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160

-- IMU 関連（3DはIMU必須）
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

-- 動きが小さいときのフィルタ
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(0.2)

return options