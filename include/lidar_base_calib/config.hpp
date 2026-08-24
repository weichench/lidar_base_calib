#ifndef LIDAR_BASE_CALIB__CONFIG_HPP_
#define LIDAR_BASE_CALIB__CONFIG_HPP_

#include <string>

namespace lidar_base_calib
{

struct InitialExtrinsic
{
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;  // approximate sensor height above ground (m, positive up)
};

struct TimeRange
{
  double start_time = 0.0;  // seconds relative to bag start
  double end_time = -1.0;   // < 0 means "until the end of the bag"
};

struct GroundPlaneParams
{
  double voxel_leaf_size = 0.1;             // m
  double ransac_distance_threshold = 0.1;   // m
  int ransac_max_iterations = 50;
  double ground_band = 0.5;                 // m, coarse ground candidate band
  bool use_initial_for_ground_filter = false;
  int min_inliers = 1000;                   // min RANSAC inliers per frame
  double max_abs_angle_deg = 85.0;          // frame-level sanity filter: rejects near-vertical
                                            // (wall) planes at ~90 deg while allowing steep
                                            // but valid mounting pitches
  double z_min = 0.05;                      // plausible height range (m)
  double z_max = 10.0;
};

struct OutputParams
{
  std::string dir = "lidar_base_calib_result";         // output directory, auto-created if missing
  std::string yaml_file = "lidar_base_calib_result.yaml";  // final result (relative to dir)
  std::string frames_file = "lidar_base_calib_frames.txt"; // per-frame results (relative to dir)
};

struct ImuParams
{
  std::string topic = "";                      // empty = disabled (no IMU-based initial guess)
  double stationary_angular_threshold = 0.1;   // rad/s; samples with |omega| above this are dropped
  double accel_magnitude_band = 0.2;           // | ||a|| - g_ref | above this are dropped (unit-agnostic)
  int min_samples = 10;                        // min near-stationary samples required
};

struct Config
{
  std::string bag_path;
  std::string lidar_topic = "/livox/lidar_192_168_1_176";
  TimeRange time_range;
  InitialExtrinsic initial_extrinsic;
  GroundPlaneParams ground_plane;
  OutputParams output;
  ImuParams imu;
};

/// Loads the unified configuration file (YAML). Returns false on failure.
bool loadConfig(const std::string & path, Config & config);

}  // namespace lidar_base_calib

#endif  // LIDAR_BASE_CALIB__CONFIG_HPP_
