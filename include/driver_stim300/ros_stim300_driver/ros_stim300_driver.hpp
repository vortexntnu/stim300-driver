#ifndef ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP
#define ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stim300_driver/driver_stim300.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

inline Quaternion from_rpy_to_quaternion(EulerAngles angles) {
  const double cy = cos(angles.yaw * 0.5);
  const double sy = sin(angles.yaw * 0.5);
  const double cp = cos(angles.pitch * 0.5);
  const double sp = sin(angles.pitch * 0.5);
  const double cr = cos(angles.roll * 0.5);
  const double sr = sin(angles.roll * 0.5);

  return {
      .w = cy * cp * cr + sy * sp * sr,
      .x = cy * cp * sr - sy * sp * cr,
      .y = sy * cp * sr + cy * sp * cr,
      .z = sy * cp * cr - cy * sp * sr,
  };
}

class Stim300DriverNode : public rclcpp::Node {
public:
  explicit Stim300DriverNode(const rclcpp::NodeOptions &options);
  ~Stim300DriverNode();

private:
  void on_status(Stim300Status status);
  void publish_measurement();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  std::string frame_id_;
  std::array<double, 3> gyro_variance_;
  std::array<double, 3> acc_variance_;

  double gravity_;
  boost::asio::io_context io_context_;
  std::unique_ptr<DriverStim300> driver_;
  std::thread io_thread_;
};

#endif // ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP
