#include "lidar_base_calib/bag_reader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_base_calib
{

namespace
{
constexpr const char * kLivoxType = "livox_ros_driver2/msg/CustomMsg";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
}  // namespace

bool BagReader::open(const std::string & bag_path, const std::string & topic,
                     double start_s, double end_s)
{
  topic_ = topic;
  start_s_ = start_s;
  end_s_ = end_s;

  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader_->open(bag_path);
  } catch (const std::exception & e) {
    std::cerr << "[bag_reader] failed to open bag '" << bag_path << "': " << e.what() << std::endl;
    reader_.reset();
    return false;
  }

  const auto metadata = reader_->get_metadata();
  bag_start_ns_ = static_cast<int64_t>(metadata.starting_time.time_since_epoch().count());

  bool found = false;
  for (const auto & topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == topic) {
      topic_type_ = topic_info.topic_metadata.type;
      found = true;
      break;
    }
  }

  if (!found) {
    std::cerr << "[bag_reader] topic '" << topic << "' not found in bag '" << bag_path << "'."
              << std::endl;
    reader_.reset();
    return false;
  }

  if (topic_type_ == kLivoxType) {
    format_ = CloudFormat::kLivoxCustomMsg;
  } else if (topic_type_ == kPointCloud2Type) {
    format_ = CloudFormat::kPointCloud2;
  } else {
    std::cerr << "[bag_reader] topic '" << topic << "' has type '" << topic_type_
              << "', expected '" << kLivoxType << "' or '" << kPointCloud2Type << "'."
              << std::endl;
    reader_.reset();
    return false;
  }

  return true;
}

bool BagReader::inRange(int64_t stamp_ns) const
{
  const double t = static_cast<double>(stamp_ns - bag_start_ns_) * 1e-9;
  if (t < start_s_) {
    return false;
  }
  if (end_s_ >= 0.0 && t >= end_s_) {
    return false;
  }
  return true;
}

bool BagReader::next(PointCloudFrame & frame)
{
  if (!reader_) {
    return false;
  }

  while (reader_->has_next()) {
    auto bag_msg = reader_->read_next();
    if (bag_msg->topic_name != topic_) {
      continue;
    }
    if (!inRange(static_cast<int64_t>(bag_msg->time_stamp))) {
      continue;
    }

    frame.stamp_ns = static_cast<int64_t>(bag_msg->time_stamp);
    frame.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    if (format_ == CloudFormat::kLivoxCustomMsg) {
      rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
      livox_ros_driver2::msg::CustomMsg msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      serialization.deserialize_message(&serialized_msg, &msg);

      frame.cloud->reserve(msg.points.size());
      for (const auto & p : msg.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = static_cast<float>(p.reflectivity);
        frame.cloud->push_back(pt);
      }
    } else {
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      serialization.deserialize_message(&serialized_msg, &msg);

      pcl::fromROSMsg(msg, *frame.cloud);

      // A PointCloud2 without an "intensity" field leaves the value uninitialized;
      // force it to 0 so it is well-defined.
      const bool has_intensity = std::any_of(
        msg.fields.begin(), msg.fields.end(),
        [](const auto & f) { return f.name == "intensity"; });
      if (!has_intensity) {
        for (auto & p : frame.cloud->points) {
          p.intensity = 0.0f;
        }
      }

      frame.cloud->erase(
        std::remove_if(
          frame.cloud->begin(), frame.cloud->end(),
          [](const pcl::PointXYZI & p) {
            return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
          }),
        frame.cloud->end());
    }

    if (frame.cloud->empty()) {
      continue;
    }

    return true;
  }

  return false;
}

}  // namespace lidar_base_calib
