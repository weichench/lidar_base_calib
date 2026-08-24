#ifndef LIDAR_BASE_CALIB__BAG_READER_HPP_
#define LIDAR_BASE_CALIB__BAG_READER_HPP_

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rosbag2_cpp/reader.hpp>

namespace lidar_base_calib
{

struct PointCloudFrame
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  int64_t stamp_ns = 0;  // receive time recorded in the bag (ns since epoch)
};

class BagReader
{
public:
  BagReader() = default;

  /// Opens the bag and verifies the target topic exists with the expected type.
  /// \param start_s / end_s  seconds relative to the bag start (end_s < 0 = to end)
  bool open(const std::string & bag_path, const std::string & topic,
            double start_s, double end_s);

  /// Reads the next frame inside the configured time range. Returns false when
  /// no more frames are available.
  bool next(PointCloudFrame & frame);

  bool isOpen() const { return reader_ != nullptr; }

private:
  bool inRange(int64_t stamp_ns) const;

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string topic_;
  double start_s_ = 0.0;
  double end_s_ = -1.0;
  int64_t bag_start_ns_ = 0;
  std::string topic_type_;
};

}  // namespace lidar_base_calib

#endif  // LIDAR_BASE_CALIB__BAG_READER_HPP_
