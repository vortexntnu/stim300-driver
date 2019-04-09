#include "driver_stim300.h"
#include "serial_unix.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


constexpr int defaultSampleRate{ 125 };
constexpr double averageAllanVarianceOfGyro{ 0.0001 * 2 * 0.00046};
constexpr double averageAllanVarianceOfAcc{ 100 * 2 * 0.0052};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node;

  std::string device_name;
  double stanardDeivationOfGyro{ 0 };
  double stanardDeviationOfAcc{ 0 };
  double varianceOfGyro{ 0 };
  double varianceOfAcc{ 0 };
  int sampleRate{ 0 };

  node.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  SerialUnix serial_driver(device_name);
  DriverStim300 driver_stim300(serial_driver);

  node.param("stanard_deviation_of_gyro", stanardDeivationOfGyro, averageAllanVarianceOfGyro);
  node.param("stanard_deviation_of_acc", stanardDeviationOfAcc, averageAllanVarianceOfAcc);
  node.param("sample_rate", sampleRate, defaultSampleRate);
  varianceOfGyro = sampleRate * pow(stanardDeivationOfGyro, 2);
  varianceOfAcc = sampleRate * pow(stanardDeviationOfAcc, 2);


  sensor_msgs::Imu imu_msg_template{};
  imu_msg_template.orientation_covariance[0] = -1;
  imu_msg_template.angular_velocity_covariance[0] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[4] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[8] = varianceOfGyro;
  imu_msg_template.linear_acceleration_covariance[0] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[4] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[8] = varianceOfAcc;
  imu_msg_template.orientation.x = 0;
  imu_msg_template.orientation.y = 0;
  imu_msg_template.orientation.z = 0;
  imu_msg_template.header.frame_id = "imu_0";

  ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Rate loop_rate(1000);

  ROS_INFO("STIM300 IMU initialized successfully");

  while (ros::ok())
  {
    sensor_msgs::Imu stim300msg = imu_msg_template;

    stim300msg.header.stamp = ros::Time::now();

    if (driver_stim300.processPacket())
    {
      if (!driver_stim300.isChecksumGood())
      {
        ROS_WARN("stim300 CRC error ");
        continue;
      }

      if (!driver_stim300.isSensorStatusGood())
      {
        ROS_WARN("STIM300: Internal hardware error");
        continue;
      }

      stim300msg.linear_acceleration.x = driver_stim300.getAccX() + 0.0023;
      stim300msg.linear_acceleration.y = driver_stim300.getAccY() + 0.05;
      stim300msg.linear_acceleration.z = driver_stim300.getAccZ() + 0.027;
      stim300msg.angular_velocity.x = driver_stim300.getGyroX();
      stim300msg.angular_velocity.y = driver_stim300.getGyroY();
      stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
      imuSensorPublisher.publish(stim300msg);

    }

    loop_rate.sleep();

    ros::spinOnce();
  }
  return 0;
}