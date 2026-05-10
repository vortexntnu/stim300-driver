#include "ros_stim300_driver/ros_stim300_driver.hpp"

Stim300DriverNode::Stim300DriverNode(const rclcpp::NodeOptions & options)
: Node("stim300_driver_node", options)
{
  declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  declare_parameter<double>("gravity", 9.80665);

  const auto device_name = get_parameter("device_name").as_string();
  const auto gravity     = get_parameter("gravity").as_double();

  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);

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

RCLCPP_COMPONENTS_REGISTER_NODE(Stim300DriverNode)
