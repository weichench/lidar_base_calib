#include "lidar_base_calib/imu_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace lidar_base_calib
{

namespace
{

constexpr const char * kExpectedType = "sensor_msgs/msg/Imu";
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

struct Sample
{
  double ax, ay, az;
  double wmag;
  double amag;
};

double medianOf(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  if (n % 2 == 1) {
    return values[n / 2];
  }
  return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

}  // namespace

ImuInitialEstimate estimateRollPitchFromImu(
    const std::string & bag_path, const std::string & imu_topic,
    double start_s, double end_s,
    double stationary_angular_threshold, double accel_magnitude_band,
    int min_samples)
{
  ImuInitialEstimate result;

  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception & e) {
    std::cerr << "[imu_estimator] failed to open bag '" << bag_path << "': "
              << e.what() << std::endl;
    return result;
  }

  const auto metadata = reader.get_metadata();
  const int64_t bag_start_ns =
    static_cast<int64_t>(metadata.starting_time.time_since_epoch().count());

  bool found = false;
  for (const auto & info : metadata.topics_with_message_count) {
    if (info.topic_metadata.name == imu_topic) {
      if (info.topic_metadata.type != kExpectedType) {
        std::cerr << "[imu_estimator] topic '" << imu_topic << "' has type '"
                  << info.topic_metadata.type << "', expected '" << kExpectedType
                  << "'." << std::endl;
        return result;
      }
      found = true;
      break;
    }
  }
  if (!found) {
    std::cerr << "[imu_estimator] topic '" << imu_topic << "' not found in bag '"
              << bag_path << "'." << std::endl;
    return result;
  }

  auto inRange = [&](int64_t stamp_ns) {
    const double t = static_cast<double>(stamp_ns - bag_start_ns) * 1e-9;
    if (t < start_s) {
      return false;
    }
    if (end_s >= 0.0 && t >= end_s) {
      return false;
    }
    return true;
  };

  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
  std::vector<Sample> samples;
  samples.reserve(8192);

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != imu_topic) {
      continue;
    }
    if (!inRange(static_cast<int64_t>(bag_msg->time_stamp))) {
      continue;
    }

    sensor_msgs::msg::Imu msg;
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    serialization.deserialize_message(&serialized_msg, &msg);

    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
      continue;
    }
    const double amag = std::sqrt(ax * ax + ay * ay + az * az);
    if (amag < 1e-6) {  // zero / uninitialized reading
      continue;
    }
    const double wx = msg.angular_velocity.x;
    const double wy = msg.angular_velocity.y;
    const double wz = msg.angular_velocity.z;
    const double wmag = std::sqrt(wx * wx + wy * wy + wz * wz);

    samples.push_back({ax, ay, az, wmag, amag});
  }

  result.total_samples = static_cast<int>(samples.size());
  if (result.total_samples < min_samples) {
    return result;
  }

  // Reference gravity magnitude = median acceleration magnitude, so the estimate
  // is independent of whether the driver reports m/s^2 or g units.
  std::vector<double> mags;
  mags.reserve(samples.size());
  for (const auto & s : samples) {
    mags.push_back(s.amag);
  }
  result.gravity_magnitude = medianOf(std::move(mags));

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  int used = 0;
  for (const auto & s : samples) {
    if (s.wmag > stationary_angular_threshold) {
      continue;
    }
    if (std::fabs(s.amag - result.gravity_magnitude) > accel_magnitude_band) {
      continue;
    }
    sum_x += s.ax;
    sum_y += s.ay;
    sum_z += s.az;
    used++;
  }
  result.used_samples = used;

  if (used < min_samples) {
    return result;
  }

  const double norm = std::sqrt(sum_x * sum_x + sum_y * sum_y + sum_z * sum_z);
  if (norm < 1e-9) {
    return result;
  }
  const double ux = sum_x / norm;
  const double uy = sum_y / norm;
  const double uz = sum_z / norm;

  // u = normalize(mean specific force) is the "up" direction, matching the ground
  // plane normal (a,b,c) convention: roll = atan2(b,c), pitch = atan2(-a, sqrt(b^2+c^2)).
  result.roll_deg = std::atan2(uy, uz) * kRad2Deg;
  result.pitch_deg = std::atan2(-ux, std::sqrt(uy * uy + uz * uz)) * kRad2Deg;
  result.valid = true;
  return result;
}

}  // namespace lidar_base_calib
