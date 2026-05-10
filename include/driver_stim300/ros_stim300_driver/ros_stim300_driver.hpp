#ifndef ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP
#define ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "stim300_driver/stim300_stream.hpp"

#include <atomic>
#include <memory>

constexpr int NUMBER_OF_CALIBRATION_SAMPLES{100};

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

struct CalibrationData {
  double inclination_x_sum{};
  double inclination_y_sum{};
  double inclination_z_sum{};
  int    n_samples{};
};

Quaternion fromRPYToQuaternion(EulerAngles angles);

class Stim300DriverNode : public rclcpp::Node {
public:
  explicit Stim300DriverNode(const rclcpp::NodeOptions & options);
  ~Stim300DriverNode();

private:
  void on_measurement(const DriverStim300 & driver);
  void on_status(Stim300Status status, const DriverStim300 & driver);
  void calibrateSensor(double inc_x, double inc_y, double inc_z);

  bool responseCalibrateIMU(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  calibration_service_;

  std::atomic<bool> calibration_mode_{false};
  CalibrationData   calibration_data_{};

  double gravity_{};
  sensor_msgs::msg::Imu stim300msg_;

  std::unique_ptr<Stim300Stream> stream_;
};

#endif // ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP
