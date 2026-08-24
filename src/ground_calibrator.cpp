#include "lidar_base_calib/ground_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Geometry>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace lidar_base_calib
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

Eigen::Matrix3d rpyToRot(double roll_rad, double pitch_rad, double yaw_rad)
{
  const Eigen::AngleAxisd rz(yaw_rad, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd ry(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rx(roll_rad, Eigen::Vector3d::UnitX());
  return (rz * ry * rx).toRotationMatrix();
}

/// Converts plane coefficients (a,b,c,d) of `a*x + b*y + c*z + d = 0` with a
/// unit normal (a,b,c) into roll/pitch (rad) and height Z (m). The normal is
/// forced to point upward (c > 0) so that Z = d is the positive sensor height.
void planeToRollPitchZ(double a, double b, double c, double d,
                       double & roll_rad, double & pitch_rad, double & z)
{
  if (c < 0.0) {
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }
  pitch_rad = std::atan2(-a, std::sqrt(b * b + c * c));
  roll_rad = std::atan2(b, c);
  z = d;
}

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

Statistics computeStatistics(const std::vector<double> & values)
{
  Statistics s;
  s.count = static_cast<int>(values.size());
  if (values.empty()) {
    return s;
  }
  s.median = medianOf(values);
  s.mean = std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
  double sq = 0.0;
  for (const double v : values) {
    sq += (v - s.mean) * (v - s.mean);
  }
  s.std = std::sqrt(sq / static_cast<double>(values.size()));
  return s;
}

}  // namespace

GroundCalibrator::GroundCalibrator(const Config & config) : config_(config)
{
  const double roll = config_.initial_extrinsic.roll_deg * kDeg2Rad;
  const double pitch = config_.initial_extrinsic.pitch_deg * kDeg2Rad;
  const double yaw = config_.initial_extrinsic.yaw_deg * kDeg2Rad;
  const Eigen::Matrix3d R0 = rpyToRot(roll, pitch, yaw);
  const Eigen::Vector3d n0 = R0.transpose() * Eigen::Vector3d::UnitZ();
  n0_x_ = n0.x();
  n0_y_ = n0.y();
  n0_z_ = n0.z();
  z0_ = config_.initial_extrinsic.z;
}

FrameResult GroundCalibrator::fitFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
{
  FrameResult result;
  if (!cloud || cloud->size() < static_cast<size_t>(config_.ground_plane.min_inliers)) {
    result.valid = false;
    return result;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    const float leaf = static_cast<float>(config_.ground_plane.voxel_leaf_size);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*downsampled);
  }

  // Optional coarse pre-filter using the expected ground plane from the initial
  // extrinsic. Falls back to the full downsampled cloud when it yields too few
  // candidates (e.g. the initial height is unknown/wrong).
  pcl::PointCloud<pcl::PointXYZI>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZI>);
  if (config_.ground_plane.use_initial_for_ground_filter) {
    const double band = config_.ground_plane.ground_band;
    candidates->reserve(downsampled->size());
    for (const auto & pt : downsampled->points) {
      const double dist = n0_x_ * pt.x + n0_y_ * pt.y + n0_z_ * pt.z + z0_;
      if (std::fabs(dist) < band) {
        candidates->push_back(pt);
      }
    }
    if (candidates->size() < static_cast<size_t>(config_.ground_plane.min_inliers)) {
      candidates = downsampled;
    }
  } else {
    candidates = downsampled;
  }

  if (candidates->size() < static_cast<size_t>(config_.ground_plane.min_inliers)) {
    result.valid = false;
    return result;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(config_.ground_plane.ransac_max_iterations);
  seg.setDistanceThreshold(config_.ground_plane.ransac_distance_threshold);
  seg.setInputCloud(candidates);
  seg.segment(*inliers, *coefficients);

  if (coefficients->values.size() < 4 ||
      static_cast<int>(inliers->indices.size()) < config_.ground_plane.min_inliers) {
    result.valid = false;
    return result;
  }

  double a = coefficients->values[0];
  double b = coefficients->values[1];
  double c = coefficients->values[2];
  double d = coefficients->values[3];

  // Normalize the normal to unit length (guards against non-unit normals).
  const double norm = std::sqrt(a * a + b * b + c * c);
  if (norm < 1e-9) {
    result.valid = false;
    return result;
  }
  a /= norm;
  b /= norm;
  c /= norm;
  d /= norm;

  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double z = 0.0;
  planeToRollPitchZ(a, b, c, d, roll_rad, pitch_rad, z);

  result.roll_deg = roll_rad * kRad2Deg;
  result.pitch_deg = pitch_rad * kRad2Deg;
  result.z = z;
  result.inliers = static_cast<int>(inliers->indices.size());

  if (std::fabs(result.roll_deg) > config_.ground_plane.max_abs_angle_deg ||
      std::fabs(result.pitch_deg) > config_.ground_plane.max_abs_angle_deg ||
      z < config_.ground_plane.z_min || z > config_.ground_plane.z_max) {
    result.valid = false;
    return result;
  }

  result.valid = true;
  return result;
}

void GroundCalibrator::addFrame(const FrameResult & frame, double stamp_s)
{
  frame_count_++;
  if (!frame.valid) {
    rejected_count_++;
    return;
  }
  rolls_.push_back(frame.roll_deg);
  pitches_.push_back(frame.pitch_deg);
  zs_.push_back(frame.z);
  frames_.push_back({stamp_s, frame.roll_deg, frame.pitch_deg, frame.z});
}

CalibrationResult GroundCalibrator::computeResult() const
{
  CalibrationResult result;
  result.frame_count = frame_count_;
  result.valid_count = static_cast<int>(rolls_.size());
  result.rejected_count = rejected_count_;
  result.roll = computeStatistics(rolls_);
  result.pitch = computeStatistics(pitches_);
  result.z = computeStatistics(zs_);
  return result;
}

}  // namespace lidar_base_calib
