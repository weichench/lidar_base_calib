#ifndef LIDAR_BASE_CALIB__IMU_ESTIMATOR_HPP_
#define LIDAR_BASE_CALIB__IMU_ESTIMATOR_HPP_

#include <string>

namespace lidar_base_calib
{

/// Result of estimating roll/pitch from the IMU gravity direction.
struct ImuInitialEstimate
{
  bool valid = false;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double gravity_magnitude = 0.0;  // reference acceleration magnitude (unit-agnostic)
  int used_samples = 0;            // near-stationary samples actually averaged
  int total_samples = 0;           // all IMU samples read in the time range
};

/// Reads the IMU topic of a bag within [start_s, end_s) seconds (relative to the
/// bag start) and estimates the sensor roll/pitch from the mean gravity direction
/// over near-stationary samples. When stationary the accelerometer measures the
/// specific force (== -gravity, i.e. the "up" direction), so the resulting
/// roll/pitch convention matches the ground-plane fit.
///
/// Returns a result with valid == false when the topic is missing, the type does
/// not match sensor_msgs/msg/Imu, or there are fewer than min_samples usable
/// near-stationary samples.
ImuInitialEstimate estimateRollPitchFromImu(
    const std::string & bag_path,
    const std::string & imu_topic,
    double start_s,
    double end_s,
    double stationary_angular_threshold,
    double accel_magnitude_band,
    int min_samples);

}  // namespace lidar_base_calib

#endif  // LIDAR_BASE_CALIB__IMU_ESTIMATOR_HPP_
