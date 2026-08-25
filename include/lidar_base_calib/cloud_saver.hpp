#ifndef LIDAR_BASE_CALIB__CLOUD_SAVER_HPP_
#define LIDAR_BASE_CALIB__CLOUD_SAVER_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_base_calib/config.hpp"
#include "lidar_base_calib/ground_calibrator.hpp"

namespace lidar_base_calib
{

/// Rigid transform from the LiDAR frame to the (level) base frame:
///   p_base = R * p_lidar + t
struct ExtrinsicTransform
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

/// Builds the calibrated extrinsic transform. roll/pitch/Z come from the
/// calibration result (median), yaw/x/y are carried through from the initial
/// extrinsic unchanged (the ground plane cannot observe them).
ExtrinsicTransform calibratedExtrinsicTransform(const CalibrationResult & result,
                                                const InitialExtrinsic & init);

/// Transforms each point of `in` into the base frame, preserving intensity.
void transformToBase(const pcl::PointCloud<pcl::PointXYZI>::Ptr & in,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr & out,
                     const ExtrinsicTransform & T);

/// Saves a point cloud to a binary PCD file. Returns false on failure.
bool saveCloudPcd(const std::string & path,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);

/// A closed time interval [first, second] in nanoseconds since epoch.
using Interval = std::pair<int64_t, int64_t>;

/// Reads the IMU topic (config.imu) and returns the stationary time intervals,
/// i.e. runs where the angular-velocity magnitude stays below the configured
/// threshold. Returns an empty vector when the IMU is disabled, missing, or has
/// the wrong type. Intervals shorter than `min_duration_ns` are dropped.
std::vector<Interval> stationaryIntervals(const Config & config, int64_t min_duration_ns);

/// True when `stamp_ns` falls inside one of the intervals (expanded by `tol_ns`
/// to absorb small lidar/imu clock offsets).
bool inStationaryInterval(int64_t stamp_ns, const std::vector<Interval> & intervals,
                          int64_t tol_ns);

}  // namespace lidar_base_calib

#endif  // LIDAR_BASE_CALIB__CLOUD_SAVER_HPP_
