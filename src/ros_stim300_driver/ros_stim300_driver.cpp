#include "ros_stim300_driver/ros_stim300_driver.hpp"

#include <cmath>

Quaternion fromRPYToQuaternion(EulerAngles angles) {
  const double cy = cos(angles.yaw   * 0.5);
  const double sy = sin(angles.yaw   * 0.5);
  const double cp = cos(angles.pitch * 0.5);
  const double sp = sin(angles.pitch * 0.5);
  const double cr = cos(angles.roll  * 0.5);
  const double sr = sin(angles.roll  * 0.5);

  return {
    .w = cy * cp * cr + sy * sp * sr,
    .x = cy * cp * sr - sy * sp * cr,
    .y = sy * cp * sr + cy * sp * cr,
    .z = sy * cp * cr - cy * sp * sr,
  };
}

Stim300DriverNode::Stim300DriverNode(const rclcpp::NodeOptions & options)
: Node("stim300_driver_node", options)
{
  declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  declare_parameter<double>("variance_gyro", 0.0001 * 2 * 4.6 * pow(10, -4));
  declare_parameter<double>("variance_acc", 0.000055);
  declare_parameter<double>("gravity", 9.80665);

  const auto device_name = get_parameter("device_name").as_string();
  gravity_ = get_parameter("gravity").as_double();

  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);
  calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "IMU_calibration",
      std::bind(&Stim300DriverNode::responseCalibrateIMU, this,
                std::placeholders::_1, std::placeholders::_2));

  stim300msg_.angular_velocity_covariance[0]    = 0.0000027474;
  stim300msg_.angular_velocity_covariance[4]    = 0.0000027474;
  stim300msg_.angular_velocity_covariance[8]    = 0.000007312;
  stim300msg_.linear_acceleration_covariance[0] = 0.00041915;
  stim300msg_.linear_acceleration_covariance[4] = 0.00041915;
  stim300msg_.linear_acceleration_covariance[8] = 0.000018995;
  stim300msg_.orientation.x                     = 0.00000024358;
  stim300msg_.orientation.y                     = 0.00000024358;
  stim300msg_.orientation.z                     = 0.00000024358;
  stim300msg_.header.frame_id                   = "imu_0";

  stream_ = std::make_unique<Stim300Stream>(
      device_name,
      [this](const DriverStim300 & driver) { on_measurement(driver); },
      [this](Stim300Status status, const DriverStim300 & driver) { on_status(status, driver); });

  RCLCPP_INFO(get_logger(), "STIM300 IMU driver initialized successfully");
}

Stim300DriverNode::~Stim300DriverNode()
{
  stream_.reset();
}

void Stim300DriverNode::on_measurement(const DriverStim300 & driver)
{
  const double inc_x = driver.getIncX();
  const double inc_y = driver.getIncY();
  const double inc_z = driver.getIncZ();

  if (calibration_mode_) {
    calibrateSensor(inc_x, inc_y, inc_z);
    return;
  }

  EulerAngles theta;
  theta.roll  = atan2(inc_y, inc_z);
  theta.pitch = atan2(-inc_x, sqrt(pow(inc_y, 2) + pow(inc_z, 2)));
  const auto q = fromRPYToQuaternion(theta);

  stim300msg_.header.stamp           = now();
  stim300msg_.linear_acceleration.x  = driver.getAccX() * gravity_;
  stim300msg_.linear_acceleration.y  = driver.getAccY() * gravity_;
  stim300msg_.linear_acceleration.z  = driver.getAccZ() * gravity_;
  stim300msg_.angular_velocity.x     = driver.getGyroX();
  stim300msg_.angular_velocity.y     = driver.getGyroY();
  stim300msg_.angular_velocity.z     = driver.getGyroZ();
  stim300msg_.orientation.w          = q.w;
  stim300msg_.orientation.x          = q.x;
  stim300msg_.orientation.y          = q.y;
  stim300msg_.orientation.z          = q.z;
  imu_publisher_->publish(stim300msg_);
}

void Stim300DriverNode::calibrateSensor(double inc_x, double inc_y, double inc_z)
{
  if (calibration_data_.n_samples < NUMBER_OF_CALIBRATION_SAMPLES) {
    calibration_data_.n_samples++;
    calibration_data_.inclination_x_sum += inc_x;
    calibration_data_.inclination_y_sum += inc_y;
    calibration_data_.inclination_z_sum += inc_z;
    return;
  }

  const double avg_x = calibration_data_.inclination_x_sum / NUMBER_OF_CALIBRATION_SAMPLES;
  const double avg_y = calibration_data_.inclination_y_sum / NUMBER_OF_CALIBRATION_SAMPLES;
  const double avg_z = calibration_data_.inclination_z_sum / NUMBER_OF_CALIBRATION_SAMPLES;

  RCLCPP_INFO(get_logger(), "roll:  %f", atan2(avg_y, avg_z));
  RCLCPP_INFO(get_logger(), "pitch: %f", atan2(-avg_x, sqrt(pow(avg_y, 2) + pow(avg_z, 2))));
  RCLCPP_INFO(get_logger(), "IMU Calibrated");
  calibration_mode_ = false;
}

void Stim300DriverNode::on_status(Stim300Status status, const DriverStim300 & driver)
{
  switch (status) {
  case Stim300Status::CONFIG_CHANGED:
    RCLCPP_INFO(get_logger(), "Updated Stim 300 imu config: %s",
                              driver.printSensorConfig().c_str());
    break;
  case Stim300Status::STARTING_SENSOR:
    RCLCPP_INFO(get_logger(), "Stim 300 IMU is warming up.");
    break;
  case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
    RCLCPP_DEBUG(get_logger(), "Stim 300 outside operating conditions");
    break;
  case Stim300Status::SYSTEM_INTEGRITY_ERROR:
    RCLCPP_WARN(get_logger(), "Stim 300 IMU system integrity error.");
    break;
  case Stim300Status::OVERLOAD:
    RCLCPP_WARN(get_logger(), "Stim 300 IMU overload.");
    break;
  case Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL:
    RCLCPP_WARN(get_logger(), "Stim 300 IMU error in measurement channel.");
    break;
  case Stim300Status::ERROR:
    RCLCPP_WARN(get_logger(), "Stim 300 IMU: internal error.");
    break;
  default:
    break;
  }
}

bool Stim300DriverNode::responseCalibrateIMU(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!calibration_mode_) {
    calibration_data_ = CalibrationData{};
    calibration_mode_ = true;
    response->message = "IMU in calibration mode";
    response->success = true;
  }
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(Stim300DriverNode)
