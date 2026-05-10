#include "ros_stim300_driver/ros_stim300_driver.hpp"

Stim300DriverNode::Stim300DriverNode(const rclcpp::NodeOptions & options)
: Node("stim300_driver_node", options)
{
  declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  declare_parameter<double>("gravity", 9.80665);
  declare_parameter<std::string>("frame_id", "imu_0");
  declare_parameter<std::vector<double>>("gyro_variance", {0.0000027474, 0.0000027474, 0.000007312});
  declare_parameter<std::vector<double>>("acc_variance",  {0.00041915,   0.00041915,   0.000018995});

  const auto device_name   = get_parameter("device_name").as_string();
  const auto gravity       = get_parameter("gravity").as_double();
  frame_id_                = get_parameter("frame_id").as_string();
  const auto gyro_var      = get_parameter("gyro_variance").as_double_array();
  const auto acc_var       = get_parameter("acc_variance").as_double_array();

  gyro_variance_ = {gyro_var[0], gyro_var[1], gyro_var[2]};
  acc_variance_  = {acc_var[0],  acc_var[1],  acc_var[2]};

  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());

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

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();

  msg->header.stamp    = rclcpp::Time(meas.stamp_ns, RCL_SYSTEM_TIME);
  msg->header.frame_id = frame_id_;

  msg->angular_velocity_covariance[0]    = gyro_variance_[0];
  msg->angular_velocity_covariance[4]    = gyro_variance_[1];
  msg->angular_velocity_covariance[8]    = gyro_variance_[2];
  msg->linear_acceleration_covariance[0] = acc_variance_[0];
  msg->linear_acceleration_covariance[4] = acc_variance_[1];
  msg->linear_acceleration_covariance[8] = acc_variance_[2];

  msg->linear_acceleration.x = meas.acc_x;
  msg->linear_acceleration.y = meas.acc_y;
  msg->linear_acceleration.z = meas.acc_z;
  msg->angular_velocity.x    = meas.gyro_x;
  msg->angular_velocity.y    = meas.gyro_y;
  msg->angular_velocity.z    = meas.gyro_z;
  msg->orientation.w         = q.w;
  msg->orientation.x         = q.x;
  msg->orientation.y         = q.y;
  msg->orientation.z         = q.z;

  imu_publisher_->publish(std::move(msg));
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
