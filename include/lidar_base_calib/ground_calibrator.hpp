#ifndef LIDAR_BASE_CALIB__GROUND_CALIBRATOR_HPP_
#define LIDAR_BASE_CALIB__GROUND_CALIBRATOR_HPP_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_base_calib/config.hpp"

namespace lidar_base_calib
{

struct FrameResult
{
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double z = 0.0;
  int inliers = 0;
  bool valid = false;
};

/// One valid per-frame calibration, with its timestamp, for the per-frame txt dump.
struct FrameRecord
{
  double stamp_s = 0.0;  // absolute timestamp in seconds since epoch
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double z = 0.0;
};

struct Statistics
{
  double median = 0.0;
  double mean = 0.0;
  double std = 0.0;
  int count = 0;
};

struct CalibrationResult
{
  Statistics roll;
  Statistics pitch;
  Statistics z;
  int frame_count = 0;     // total frames fed
  int valid_count = 0;     // frames that produced a valid fit
  int rejected_count = 0;  // frames rejected by the sanity filter
};

class GroundCalibrator
{
public:
  explicit GroundCalibrator(const Config & config);

  /// Fits the ground plane of a single frame. The returned FrameResult has
  /// valid == false when the fit is degenerate or fails the sanity checks.
  FrameResult fitFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud);

  void addFrame(const FrameResult & frame, double stamp_s);

  CalibrationResult computeResult() const;

  /// Per-frame results (valid frames only), in processing order.
  const std::vector<FrameRecord> & frames() const { return frames_; }

private:
  Config config_;
  std::vector<double> rolls_;
  std::vector<double> pitches_;
  std::vector<double> zs_;
  std::vector<FrameRecord> frames_;
  int frame_count_ = 0;
  int rejected_count_ = 0;

  // Expected ground plane in the lidar frame, derived from the initial extrinsic:
  //   normal n0 = R0^T * [0,0,1] (unit, up), plane n0 . p + z0 = 0.
  double n0_x_ = 0.0;
  double n0_y_ = 0.0;
  double n0_z_ = 1.0;
  double z0_ = 0.0;
};

}  // namespace lidar_base_calib

#endif  // LIDAR_BASE_CALIB__GROUND_CALIBRATOR_HPP_
