#include "driver_stim300.h"
#include "serial_unix.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node;

  std::string device_name;
  double variance_gyro{ 0 };
  double variance_acc{ 0 };
  int sample_rate{ 0 };
  double gravity{ 0 };

  node.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  node.param("variance_gyro", variance_gyro,0.0001);
  node.param("variance_acc", variance_acc, 4.0);
  node.param("sample_rate", sample_rate, 125);
  node.param("gravity", gravity, 9.80665);

  sensor_msgs::Imu stim300msg{};
  stim300msg.orientation_covariance[0] = -1;
  stim300msg.angular_velocity_covariance[0] = variance_gyro;
  stim300msg.angular_velocity_covariance[4] = variance_gyro;
  stim300msg.angular_velocity_covariance[8] = variance_gyro;
  stim300msg.linear_acceleration_covariance[0] = variance_acc;
  stim300msg.linear_acceleration_covariance[4] = variance_acc;
  stim300msg.linear_acceleration_covariance[8] = variance_acc;
  stim300msg.orientation.x = 0;
  stim300msg.orientation.y = 0;
  stim300msg.orientation.z = 0;
  stim300msg.header.frame_id = "imu_0";

  ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  //ros::ServiceServer service = node.advertiseService("IMU_calibration",responseCalibrateIMU);

  // New messages are sent from the sensor with sample_rate
  // As loop_rate determines how often we check for new data
  // on the serial buffer, theoretically loop_rate = sample_rate
  // should be okey, but to be sure we double it
  ros::Rate loop_rate(sample_rate * 2);

  try
  {
    SerialUnix serial_driver(device_name, stim_const::BaudRate::BAUD_921600);
    DriverStim300 driver_stim300(serial_driver);


    ROS_INFO("STIM300 IMU driver initialized successfully");

    while (ros::ok())
    {
      switch (driver_stim300.update())
      {
        case Stim300Status::NORMAL:
          break;
        case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
          ROS_DEBUG("Stim 300 outside operating conditions");
        case Stim300Status::NEW_MEASURMENT:
          stim300msg.header.stamp = ros::Time::now();
          stim300msg.linear_acceleration.x = driver_stim300.getAccX() * gravity;
          stim300msg.linear_acceleration.y = driver_stim300.getAccY() * gravity;
          stim300msg.linear_acceleration.z = driver_stim300.getAccZ() * gravity;
          stim300msg.angular_velocity.x = driver_stim300.getGyroX();
          stim300msg.angular_velocity.y = driver_stim300.getGyroY();
          stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
          imuSensorPublisher.publish(stim300msg);
          break;
        case Stim300Status::CONFIG_CHANGED:
          ROS_INFO("Updated Stim 300 imu config: ");
          ROS_INFO("%s", driver_stim300.printSensorConfig().c_str());
          loop_rate = driver_stim300.getSampleRate()*2;
          break;
        case Stim300Status::STARTING_SENSOR:
          ROS_INFO("Stim 300 IMU is warming up.");
          break;
        case Stim300Status::SYSTEM_INTEGRITY_ERROR:
          ROS_WARN("Stim 300 IMU system integrity error.");
          break;
        case Stim300Status::OVERLOAD:
          ROS_WARN("Stim 300 IMU overload.");
          break;
        case Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL:
          ROS_WARN("Stim 300 IMU error in measurement channel.");
          break;
        case Stim300Status::ERROR:
          ROS_WARN("Stim 300 IMU: internal error.");
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
    return 0;
  }
  catch (std::runtime_error& error)
  {
    // TODO: Reset IMU 
    ROS_ERROR("%s\n", error.what());
    return 0;
  }
}