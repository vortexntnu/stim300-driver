#include "ros_stim300_driver/ros_stim300_driver.hpp"


Stim300DriverNode::Stim300DriverNode(const rclcpp::NodeOptions & options)
: Node("stim300_driver_node", options)
{
  declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  declare_parameter<double>("variance_gyro", 0.0001 * 2 * 4.6 * pow(10, -4));
  declare_parameter<double>("variance_acc", 0.000055);
  declare_parameter<double>("gravity", 9.80665);

  const auto device_name = get_parameter("device_name").as_string();
  const auto gravity     = get_parameter("gravity").as_double();

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
      gravity,
      [this](const ImuMeasurement & meas)                     { on_measurement(meas); },
      [this](Stim300Status status, const DriverStim300 & drv) { on_status(status, drv); });

  RCLCPP_INFO(get_logger(), "STIM300 IMU driver initialized successfully");
}

Stim300DriverNode::~Stim300DriverNode()
{
  stream_.reset();
}

void Stim300DriverNode::on_measurement(const ImuMeasurement & meas)
{
  if (calibration_mode_) {
    calibrateSensor(meas);
    return;
  }

  EulerAngles theta;
  theta.roll  = atan2(meas.inc_y, meas.inc_z);
  theta.pitch = atan2(-meas.inc_x, sqrt(pow(meas.inc_y, 2) + pow(meas.inc_z, 2)));
  const auto q = fromRPYToQuaternion(theta);

  stim300msg_.header.stamp          = rclcpp::Time(meas.stamp_ns, RCL_SYSTEM_TIME);
  stim300msg_.linear_acceleration.x = meas.acc_x;
  stim300msg_.linear_acceleration.y = meas.acc_y;
  stim300msg_.linear_acceleration.z = meas.acc_z;
  stim300msg_.angular_velocity.x    = meas.gyro_x;
  stim300msg_.angular_velocity.y    = meas.gyro_y;
  stim300msg_.angular_velocity.z    = meas.gyro_z;
  stim300msg_.orientation.w         = q.w;
  stim300msg_.orientation.x         = q.x;
  stim300msg_.orientation.y         = q.y;
  stim300msg_.orientation.z         = q.z;
  imu_publisher_->publish(stim300msg_);
}

void Stim300DriverNode::calibrateSensor(const ImuMeasurement & meas)
{
  if (calibration_data_.n_samples < NUMBER_OF_CALIBRATION_SAMPLES) {
    calibration_data_.n_samples++;
    calibration_data_.inclination_x_sum += meas.inc_x;
    calibration_data_.inclination_y_sum += meas.inc_y;
    calibration_data_.inclination_z_sum += meas.inc_z;
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
