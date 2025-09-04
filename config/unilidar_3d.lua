-- Cartographer 3D configuration for ROS 2 (Humble-compatible)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames
  map_frame = "map",
  tracking_frame = "unilidar_imu",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  -- Sensors
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- Lidar topics (3D uses PointCloud2)
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1, -- harmless in 3D
  num_point_clouds = 1,

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

-- Use 3D builder
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

-- Pose graph / loop closure (safe overrides)
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.0
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight   = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight       = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight          = 1e2

-- 3D builder
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- Keep defaults and override only needed Ceres scan matcher fields
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Do NOT replace the whole submaps table; override specific fields
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160

-- IMU is required in 3D
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

TRAJECTORY_BUILDER_3D.motion_filter = {
  max_time_seconds = 0.5,
  max_distance_meters = 0.1,
  max_angle_radians = math.rad(0.2),
}

return options