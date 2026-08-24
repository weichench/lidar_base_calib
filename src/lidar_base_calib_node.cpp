#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include "lidar_base_calib/bag_reader.hpp"
#include "lidar_base_calib/config.hpp"
#include "lidar_base_calib/ground_calibrator.hpp"
#include "lidar_base_calib/imu_estimator.hpp"

using lidar_base_calib::BagReader;
using lidar_base_calib::CalibrationResult;
using lidar_base_calib::Config;
using lidar_base_calib::FrameRecord;
using lidar_base_calib::FrameResult;
using lidar_base_calib::GroundCalibrator;
using lidar_base_calib::ImuInitialEstimate;
using lidar_base_calib::PointCloudFrame;
using lidar_base_calib::Statistics;

namespace
{

void printUsage(const char * prog)
{
  std::cerr << "Usage: " << prog << " <config.yaml>\n"
            << "  Reads all parameters from a unified YAML config file at startup.\n";
}

void writeStat(YAML::Emitter & out, const std::string & key, const Statistics & s)
{
  out << YAML::Key << key << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "median" << YAML::Value << s.median;
  out << YAML::Key << "mean" << YAML::Value << s.mean;
  out << YAML::Key << "std" << YAML::Value << s.std;
  out << YAML::EndMap;
}

bool ensureOutputDir(const std::string & dir)
{
  if (dir.empty()) {
    return true;
  }
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::cerr << "[main] failed to create output directory '" << dir << "': "
              << ec.message() << std::endl;
    return false;
  }
  return true;
}

void writeFramesTxt(const std::string & path, const std::vector<FrameRecord> & frames)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "[main] failed to open frames file: " << path << std::endl;
    return;
  }
  file << "# timestamp_s(epoch seconds) roll_deg pitch_deg z_m\n";
  file << std::fixed << std::setprecision(6);
  for (const auto & f : frames) {
    file << f.stamp_s << " " << f.roll_deg << " " << f.pitch_deg << " " << f.z << "\n";
  }
  file.close();
  std::cout << "[main] per-frame results written to " << path
            << " (" << frames.size() << " frames)\n";
}

void writeResultYaml(const std::string & path, const Config & config,
                     const CalibrationResult & result,
                     const ImuInitialEstimate & imu_est)
{
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "lidar_base_calib" << YAML::Value << YAML::BeginMap;

  out << YAML::Key << "topic" << YAML::Value << config.lidar_topic;
  out << YAML::Key << "frame_count" << YAML::Value << result.frame_count;
  out << YAML::Key << "rejected_frames" << YAML::Value << result.rejected_count;

  out << YAML::Key << "initial_extrinsic" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "roll_deg" << YAML::Value << config.initial_extrinsic.roll_deg;
  out << YAML::Key << "pitch_deg" << YAML::Value << config.initial_extrinsic.pitch_deg;
  out << YAML::Key << "yaw_deg" << YAML::Value << config.initial_extrinsic.yaw_deg;
  out << YAML::Key << "x" << YAML::Value << config.initial_extrinsic.x;
  out << YAML::Key << "y" << YAML::Value << config.initial_extrinsic.y;
  out << YAML::Key << "z" << YAML::Value << config.initial_extrinsic.z;
  out << YAML::EndMap;

  out << YAML::Key << "calibrated_extrinsic" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "roll_deg" << YAML::Value << result.roll.median;
  out << YAML::Key << "pitch_deg" << YAML::Value << result.pitch.median;
  out << YAML::Key << "yaw_deg" << YAML::Value << config.initial_extrinsic.yaw_deg;
  out << YAML::Key << "x" << YAML::Value << config.initial_extrinsic.x;
  out << YAML::Key << "y" << YAML::Value << config.initial_extrinsic.y;
  out << YAML::Key << "z" << YAML::Value << result.z.median;
  out << YAML::EndMap;

  out << YAML::Key << "statistics" << YAML::Value << YAML::BeginMap;
  writeStat(out, "roll_deg", result.roll);
  writeStat(out, "pitch_deg", result.pitch);
  writeStat(out, "z_m", result.z);
  out << YAML::EndMap;

  out << YAML::Key << "imu_initial" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "enabled" << YAML::Value << !config.imu.topic.empty();
  out << YAML::Key << "valid" << YAML::Value << imu_est.valid;
  out << YAML::Key << "roll_deg" << YAML::Value << imu_est.roll_deg;
  out << YAML::Key << "pitch_deg" << YAML::Value << imu_est.pitch_deg;
  out << YAML::Key << "gravity_magnitude" << YAML::Value << imu_est.gravity_magnitude;
  out << YAML::Key << "used_samples" << YAML::Value << imu_est.used_samples;
  out << YAML::Key << "total_samples" << YAML::Value << imu_est.total_samples;
  out << YAML::EndMap;

  out << YAML::EndMap << YAML::EndMap;

  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "[main] failed to open output file: " << path << std::endl;
    return;
  }
  file << out.c_str() << "\n";
  file.close();
  std::cout << "[main] result written to " << path << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }
  const std::string config_path = argv[1];

  rclcpp::init(argc, argv);

  Config config;
  if (!lidar_base_calib::loadConfig(config_path, config)) {
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("lidar_base_calib");

  BagReader reader;
  if (!reader.open(config.bag_path, config.lidar_topic,
                   config.time_range.start_time, config.time_range.end_time)) {
    rclcpp::shutdown();
    return 1;
  }

  // Optional: seed the initial roll/pitch from the IMU gravity direction instead
  // of the manually-filled values. This makes the ground-plane coarse filter
  // (when ground_plane.use_initial_for_ground_filter is true) more accurate for
  // steep mountings. On any failure (missing topic, wrong type, too few samples,
  // or an exception) we keep the manually-configured initial_extrinsic.
  ImuInitialEstimate imu_est;
  if (!config.imu.topic.empty()) {
    try {
      imu_est = lidar_base_calib::estimateRollPitchFromImu(
        config.bag_path, config.imu.topic,
        config.time_range.start_time, config.time_range.end_time,
        config.imu.stationary_angular_threshold,
        config.imu.accel_magnitude_band,
        config.imu.min_samples);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "IMU initial guess threw: %s", e.what());
      imu_est.valid = false;
    }
    if (imu_est.valid) {
      config.initial_extrinsic.roll_deg = imu_est.roll_deg;
      config.initial_extrinsic.pitch_deg = imu_est.pitch_deg;
      RCLCPP_INFO(node->get_logger(),
        "IMU initial guess: roll=%.3f deg, pitch=%.3f deg (%d/%d stationary samples)",
        imu_est.roll_deg, imu_est.pitch_deg, imu_est.used_samples, imu_est.total_samples);
    } else {
      RCLCPP_WARN(node->get_logger(),
        "IMU initial guess unavailable; keeping config initial_extrinsic "
        "(roll=%.2f deg, pitch=%.2f deg)",
        config.initial_extrinsic.roll_deg, config.initial_extrinsic.pitch_deg);
    }
  }

  GroundCalibrator calibrator(config);

  RCLCPP_INFO(node->get_logger(), "calibrating topic '%s' from bag '%s'",
              config.lidar_topic.c_str(), config.bag_path.c_str());

  PointCloudFrame frame;
  size_t processed = 0;
  while (reader.next(frame)) {
    const FrameResult fr = calibrator.fitFrame(frame.cloud);
    const double stamp_s = static_cast<double>(frame.stamp_ns) * 1e-9;
    calibrator.addFrame(fr, stamp_s);
    processed++;
    if (processed % 50 == 0) {
      RCLCPP_INFO(node->get_logger(), "processed %zu frames", processed);
    }
  }

  const CalibrationResult result = calibrator.computeResult();

  std::cout << "\n=== lidar_base_calib result ===\n";
  std::cout << "topic        : " << config.lidar_topic << "\n";
  std::cout << "frame_count  : " << result.frame_count
            << " (valid " << result.valid_count
            << ", rejected " << result.rejected_count << ")\n";
  std::cout << "roll  [deg]  : median " << result.roll.median
            << "  mean " << result.roll.mean
            << "  std " << result.roll.std << "\n";
  std::cout << "pitch [deg]  : median " << result.pitch.median
            << "  mean " << result.pitch.mean
            << "  std " << result.pitch.std << "\n";
  std::cout << "Z     [m]    : median " << result.z.median
            << "  mean " << result.z.mean
            << "  std " << result.z.std << "\n";
  std::cout << "===============================\n" << std::endl;

  if (result.valid_count == 0) {
    RCLCPP_WARN(node->get_logger(),
                "no valid ground fits; check initial_extrinsic / ground_plane params");
  }

  if (!ensureOutputDir(config.output.dir)) {
    RCLCPP_WARN(node->get_logger(), "could not create output directory '%s'; results will be "
                "written to the current working directory", config.output.dir.c_str());
  }

  const std::filesystem::path out_dir =
    config.output.dir.empty() ? std::filesystem::path(".") : std::filesystem::path(config.output.dir);
  const std::string yaml_path = (out_dir / config.output.yaml_file).string();
  const std::string frames_path = (out_dir / config.output.frames_file).string();

  writeResultYaml(yaml_path, config, result, imu_est);
  writeFramesTxt(frames_path, calibrator.frames());

  rclcpp::shutdown();
  return 0;
}
