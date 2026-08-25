#include "lidar_base_calib/cloud_saver.hpp"

#include <cmath>
#include <iostream>

#include <pcl/io/pcd_io.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace lidar_base_calib
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr const char * kImuType = "sensor_msgs/msg/Imu";

Eigen::Matrix3d rpyToRot(double roll_rad, double pitch_rad, double yaw_rad)
{
  const Eigen::AngleAxisd rz(yaw_rad, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd ry(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rx(roll_rad, Eigen::Vector3d::UnitX());
  return (rz * ry * rx).toRotationMatrix();
}

}  // namespace

ExtrinsicTransform calibratedExtrinsicTransform(const CalibrationResult & result,
                                                const InitialExtrinsic & init)
{
  ExtrinsicTransform T;
  T.R = rpyToRot(result.roll.median * kDeg2Rad, result.pitch.median * kDeg2Rad,
                 init.yaw_deg * kDeg2Rad);
  T.t = Eigen::Vector3d(init.x, init.y, result.z.median);
  return T;
}

void transformToBase(const pcl::PointCloud<pcl::PointXYZI>::Ptr & in,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr & out,
                     const ExtrinsicTransform & T)
{
  out->clear();
  out->reserve(in->size());
  for (const auto & p : in->points) {
    const Eigen::Vector3d q = T.R * Eigen::Vector3d(p.x, p.y, p.z) + T.t;
    pcl::PointXYZI pt;
    pt.x = static_cast<float>(q.x());
    pt.y = static_cast<float>(q.y());
    pt.z = static_cast<float>(q.z());
    pt.intensity = p.intensity;
    out->push_back(pt);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  out->is_dense = in->is_dense;
}

bool saveCloudPcd(const std::string & path,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  if (!cloud || cloud->empty()) {
    std::cerr << "[cloud_saver] refusing to save empty cloud: " << path << std::endl;
    return false;
  }
  if (pcl::io::savePCDFileBinary(path, *cloud) < 0) {
    std::cerr << "[cloud_saver] failed to save PCD: " << path << std::endl;
    return false;
  }
  return true;
}

std::vector<Interval> stationaryIntervals(const Config & config, int64_t min_duration_ns)
{
  std::vector<Interval> intervals;
  if (config.imu.topic.empty()) {
    return intervals;
  }

  rosbag2_cpp::Reader reader;
  try {
    reader.open(config.bag_path);
  } catch (const std::exception & e) {
    std::cerr << "[cloud_saver] failed to open bag for IMU: " << e.what() << std::endl;
    return intervals;
  }

  const auto metadata = reader.get_metadata();
  const int64_t bag_start_ns =
    static_cast<int64_t>(metadata.starting_time.time_since_epoch().count());

  bool found = false;
  for (const auto & info : metadata.topics_with_message_count) {
    if (info.topic_metadata.name == config.imu.topic) {
      if (info.topic_metadata.type != kImuType) {
        std::cerr << "[cloud_saver] IMU topic '" << config.imu.topic
                  << "' has type '" << info.topic_metadata.type
                  << "', expected '" << kImuType << "'; skipping stationary detection."
                  << std::endl;
        return intervals;
      }
      found = true;
      break;
    }
  }
  if (!found) {
    std::cerr << "[cloud_saver] IMU topic '" << config.imu.topic
              << "' not found; skipping stationary detection." << std::endl;
    return intervals;
  }

  const double start_s = config.time_range.start_time;
  const double end_s = config.time_range.end_time;
  const double threshold = config.imu.stationary_angular_threshold;

  auto inRange = [&](int64_t ns) {
    const double t = static_cast<double>(ns - bag_start_ns) * 1e-9;
    if (t < start_s) {
      return false;
    }
    if (end_s >= 0.0 && t >= end_s) {
      return false;
    }
    return true;
  };

  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
  Interval open{-1, -1};

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != config.imu.topic) {
      continue;
    }
    const int64_t ns = static_cast<int64_t>(bag_msg->time_stamp);
    if (!inRange(ns)) {
      continue;
    }

    sensor_msgs::msg::Imu msg;
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    serialization.deserialize_message(&serialized_msg, &msg);

    const double wx = msg.angular_velocity.x;
    const double wy = msg.angular_velocity.y;
    const double wz = msg.angular_velocity.z;
    const double wmag = std::sqrt(wx * wx + wy * wy + wz * wz);
    const bool is_static = wmag <= threshold;

    if (is_static) {
      if (open.first < 0) {
        open = {ns, ns};
      } else {
        open.second = ns;
      }
    } else {
      if (open.first >= 0 && open.second - open.first >= min_duration_ns) {
        intervals.push_back(open);
      }
      open = {-1, -1};
    }
  }

  if (open.first >= 0 && open.second - open.first >= min_duration_ns) {
    intervals.push_back(open);
  }

  return intervals;
}

bool inStationaryInterval(int64_t stamp_ns, const std::vector<Interval> & intervals,
                          int64_t tol_ns)
{
  for (const auto & iv : intervals) {
    if (stamp_ns >= iv.first - tol_ns && stamp_ns <= iv.second + tol_ns) {
      return true;
    }
  }
  return false;
}

}  // namespace lidar_base_calib
