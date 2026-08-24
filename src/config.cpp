#include "lidar_base_calib/config.hpp"

#include <iostream>

#include <yaml-cpp/yaml.h>

namespace lidar_base_calib
{

namespace
{

bool hasKey(const YAML::Node & node, const std::string & key)
{
  return node && node.IsMap() && node[key];
}

template <typename T>
T get(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (hasKey(node, key)) {
    return node[key].as<T>();
  }
  return fallback;
}

}  // namespace

bool loadConfig(const std::string & path, Config & config)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
      std::cerr << "[config] empty or invalid YAML file: " << path << std::endl;
      return false;
    }

    config.bag_path = get<std::string>(root, "bag_path", config.bag_path);
    config.lidar_topic = get<std::string>(root, "lidar_topic", config.lidar_topic);

    if (hasKey(root, "time_range")) {
      const YAML::Node & tr = root["time_range"];
      config.time_range.start_time = get<double>(tr, "start_time", config.time_range.start_time);
      config.time_range.end_time = get<double>(tr, "end_time", config.time_range.end_time);
    }

    if (hasKey(root, "initial_extrinsic")) {
      const YAML::Node & ie = root["initial_extrinsic"];
      config.initial_extrinsic.roll_deg =
        get<double>(ie, "roll_deg", config.initial_extrinsic.roll_deg);
      config.initial_extrinsic.pitch_deg =
        get<double>(ie, "pitch_deg", config.initial_extrinsic.pitch_deg);
      config.initial_extrinsic.yaw_deg =
        get<double>(ie, "yaw_deg", config.initial_extrinsic.yaw_deg);
      config.initial_extrinsic.x = get<double>(ie, "x", config.initial_extrinsic.x);
      config.initial_extrinsic.y = get<double>(ie, "y", config.initial_extrinsic.y);
      config.initial_extrinsic.z = get<double>(ie, "z", config.initial_extrinsic.z);
    }

    if (hasKey(root, "ground_plane")) {
      const YAML::Node & gp = root["ground_plane"];
      config.ground_plane.voxel_leaf_size =
        get<double>(gp, "voxel_leaf_size", config.ground_plane.voxel_leaf_size);
      config.ground_plane.ransac_distance_threshold =
        get<double>(gp, "ransac_distance_threshold", config.ground_plane.ransac_distance_threshold);
      config.ground_plane.ransac_max_iterations =
        get<int>(gp, "ransac_max_iterations", config.ground_plane.ransac_max_iterations);
      config.ground_plane.ground_band =
        get<double>(gp, "ground_band", config.ground_plane.ground_band);
      config.ground_plane.use_initial_for_ground_filter =
        get<bool>(gp, "use_initial_for_ground_filter", config.ground_plane.use_initial_for_ground_filter);
      config.ground_plane.min_inliers =
        get<int>(gp, "min_inliers", config.ground_plane.min_inliers);
      config.ground_plane.max_abs_angle_deg =
        get<double>(gp, "max_abs_angle_deg", config.ground_plane.max_abs_angle_deg);
      if (hasKey(gp, "z_range") && gp["z_range"].IsSequence() && gp["z_range"].size() >= 2) {
        config.ground_plane.z_min = gp["z_range"][0].as<double>();
        config.ground_plane.z_max = gp["z_range"][1].as<double>();
      }
    }

    if (hasKey(root, "output")) {
      const YAML::Node & out = root["output"];
      config.output.dir = get<std::string>(out, "dir", config.output.dir);
      config.output.yaml_file = get<std::string>(out, "yaml_file", config.output.yaml_file);
      config.output.frames_file = get<std::string>(out, "frames_file", config.output.frames_file);
    }

    if (hasKey(root, "imu")) {
      const YAML::Node & im = root["imu"];
      config.imu.topic = get<std::string>(im, "topic", config.imu.topic);
      config.imu.stationary_angular_threshold =
        get<double>(im, "stationary_angular_threshold", config.imu.stationary_angular_threshold);
      config.imu.accel_magnitude_band =
        get<double>(im, "accel_magnitude_band", config.imu.accel_magnitude_band);
      config.imu.min_samples = get<int>(im, "min_samples", config.imu.min_samples);
    }

    if (config.bag_path.empty()) {
      std::cerr << "[config] 'bag_path' is required but not set in " << path << std::endl;
      return false;
    }

    return true;
  } catch (const std::exception & e) {
    std::cerr << "[config] failed to parse " << path << ": " << e.what() << std::endl;
    return false;
  }
}

}  // namespace lidar_base_calib
