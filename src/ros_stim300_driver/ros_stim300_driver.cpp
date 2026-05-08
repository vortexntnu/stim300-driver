#include "ros_stim300_driver/ros_stim300_driver.hpp"

Quaternion fromRPYToQuaternion(EulerAngles angles) {
  double cy = cos(angles.yaw * 0.5);
  double sy = sin(angles.yaw * 0.5);
  double cp = cos(angles.pitch * 0.5);
  double sp = sin(angles.pitch * 0.5);
  double cr = cos(angles.roll * 0.5);
  double sr = sin(angles.roll * 0.5);

  Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

Stim300DriverNode::Stim300DriverNode()
    : Node("stim300_driver_node") 
{
  this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  this->declare_parameter<double>("variance_gyro",
                                  0.0001 * 2 * 4.6 * pow(10, -4));
  this->declare_parameter<double>("variance_acc", 0.000055);
  this->declare_parameter<int>("sample_rate", 125);
  this->declare_parameter<double>("gravity", 9.80665);

  this->get_parameter("device_name", device_name_);
  this->get_parameter("variance_gyro", variance_gyro_);
  this->get_parameter("variance_acc", variance_acc_);
  this->get_parameter("sample_rate", sample_rate_);
  this->get_parameter("gravity", gravity_);

  imu_publisher_ =
      this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);
  calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
      "IMU_calibration",
      std::bind(&Stim300DriverNode::responseCalibrateIMU, this,
                std::placeholders::_1, std::placeholders::_2));

  stim300msg_.angular_velocity_covariance[0] = 0.0000027474;
  stim300msg_.angular_velocity_covariance[4] = 0.0000027474;
  stim300msg_.angular_velocity_covariance[8] = 0.000007312;
  stim300msg_.linear_acceleration_covariance[0] = 0.00041915;
  stim300msg_.linear_acceleration_covariance[4] = 0.00041915;
  stim300msg_.linear_acceleration_covariance[8] = 0.000018995;
  stim300msg_.orientation.x = 0.00000024358;
  stim300msg_.orientation.y = 0.00000024358;
  stim300msg_.orientation.z = 0.00000024358;
  stim300msg_.header.frame_id = "imu_0";

  serial_driver_ = std::make_unique<SerialUnix>(
      device_name_, stim_const::BaudRate::BAUD_921600);
  driver_stim300_ = std::make_unique<DriverStim300>(*serial_driver_);
  RCLCPP_INFO(this->get_logger(),
              "STIM300 IMU driver initialized successfully");

  setupTimedCallback();
}

void Stim300DriverNode::setupTimedCallback() 
{
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(
          static_cast<int>(1000.0 / (sample_rate_ * 2))),
      std::bind(&Stim300DriverNode::timerCallback, this));
}

// TODO: Why is calibration data not used after running this sequence?
void Stim300DriverNode::calibrateSensor(const int inc_x, const int inc_y,
                                        const int inc_z)
{
  if (calibrationData_.n_samples < NUMBER_OF_CALIBRATION_SAMPLES) {
    calibrationData_.n_samples++;
    calibrationData_.inclination_x_sum += inc_x;
    calibrationData_.inclination_y_sum += inc_y;
    calibrationData_.inclination_z_sum += inc_z;
    return;
  }

  const double inclination_x_average =
      calibrationData_.inclination_x_sum / NUMBER_OF_CALIBRATION_SAMPLES;
  const double inclination_y_average =
      calibrationData_.inclination_y_sum / NUMBER_OF_CALIBRATION_SAMPLES;
  const double inclination_z_average =
      calibrationData_.inclination_z_sum / NUMBER_OF_CALIBRATION_SAMPLES;

  const double average_calibration_roll =
      atan2(inclination_y_average, inclination_z_average);
  const double average_calibration_pitch =
      atan2(-inclination_x_average, sqrt(pow(inclination_y_average, 2) +
                                          pow(inclination_z_average, 2)));
  std::cout << average_calibration_roll << std::endl;
  std::cout << average_calibration_pitch << std::endl;
  RCLCPP_INFO(this->get_logger(), "roll: %f", average_calibration_roll);
  RCLCPP_INFO(this->get_logger(), "pitch: %f", average_calibration_pitch);
  RCLCPP_INFO(this->get_logger(), "IMU Calibrated");
  calibration_mode_ = false;
}

void Stim300DriverNode::processNewMeasurement()
{
  const double inclination_x = driver_stim300_->getIncX();
  const double inclination_y = driver_stim300_->getIncY();
  const double inclination_z = driver_stim300_->getIncZ();
  if (calibration_mode_ == true) {
    calibrateSensor(inclination_x, inclination_y, inclination_z);
    return;
  }


  Quaternion q;
  EulerAngles theta;

  theta.roll = atan2(inclination_y, inclination_z);
  theta.pitch = atan2(-inclination_x,sqrt(pow(inclination_y, 2)+pow(inclination_z, 2)));
  q = fromRPYToQuaternion(theta);

  stim300msg_.header.stamp = this->now();
  stim300msg_.linear_acceleration.x = driver_stim300_->getAccX() * gravity_;
  stim300msg_.linear_acceleration.y = driver_stim300_->getAccY() * gravity_;
  stim300msg_.linear_acceleration.z = driver_stim300_->getAccZ() * gravity_;
  stim300msg_.angular_velocity.x = driver_stim300_->getGyroX();
  stim300msg_.angular_velocity.y = driver_stim300_->getGyroY();
  stim300msg_.angular_velocity.z = driver_stim300_->getGyroZ();
  stim300msg_.orientation.w = q.w;
  stim300msg_.orientation.x = q.x;
  stim300msg_.orientation.y = q.y;
  stim300msg_.orientation.z = q.z;
  imu_publisher_->publish(stim300msg_);
}

void Stim300DriverNode::timerCallback() {

  switch (driver_stim300_->update()) {
  case Stim300Status::NORMAL:
    break;
  case Stim300Status::NEW_MEASURMENT:
    processNewMeasurement();
    break;
  case Stim300Status::CONFIG_CHANGED:
    RCLCPP_INFO(this->get_logger(), "Updated Stim 300 imu config: ");
    RCLCPP_INFO(this->get_logger(), "%s",
                driver_stim300_->printSensorConfig().c_str());
    sample_rate_ = driver_stim300_->getSampleRate();
    timer_->cancel();
    setupTimedCallback();
    break;
  case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
    RCLCPP_DEBUG(this->get_logger(), "Stim 300 outside operating conditions");
    break;
  case Stim300Status::STARTING_SENSOR:
    RCLCPP_INFO(this->get_logger(), "Stim 300 IMU is warming up.");
    break;
  case Stim300Status::SYSTEM_INTEGRITY_ERROR:
    RCLCPP_WARN(this->get_logger(), "Stim 300 IMU system integrity error.");
    break;
  case Stim300Status::OVERLOAD:
    RCLCPP_WARN(this->get_logger(), "Stim 300 IMU overload.");
    break;
  case Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL:
    RCLCPP_WARN(this->get_logger(),
                "Stim 300 IMU error in measurement channel.");
    break;
  case Stim300Status::ERROR:
    RCLCPP_WARN(this->get_logger(), "Stim 300 IMU: internal error.");
  }
}

bool Stim300DriverNode::responseCalibrateIMU(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!calibration_mode_) {
    calibration_mode_ = true;
    response->message = "IMU in calibration mode";
    response->success = true;
  }

  return true;
}
