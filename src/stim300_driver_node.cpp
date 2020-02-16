#include "driver_stim300.h"
#include "serial_unix.h"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "iostream"

bool calibration_mode{false};
constexpr int NUMBER_OF_CALIBRATION_SAMPLES{100};
struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

Quaternion FromRPYToQuaternion(EulerAngles angles) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
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



bool responseCalibrateIMU(std_srvs::Trigger::Request &calibration_request, std_srvs::Trigger::Response &calibration_response)
{

    if (calibration_mode == false)
    {
        calibration_mode = true;
        calibration_response.message = "IMU in calibration mode ";
        calibration_response.success = true;
    }
    
    return true;
}


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
  node.param("variance_gyro", variance_gyro,0.0001*2*4.6*pow(10,-4));
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
  ros::Publisher orientationPublisher = node.advertise<sensor_msgs::Imu>("imu/orientation", 1000);
  ros::ServiceServer service = node.advertiseService("IMU_calibration",responseCalibrateIMU);


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

    int difference_in_dataGram{0};
    int count_messages{0};
    int number_of_samples{0};
    double inclination_x{0};
    double inclination_y{0};
    double inclination_z{0};

    double average_calibration_roll{0};
    double average_calibration_pitch{0};
    double inclination_x_calibration_sum{0};
    double inclination_y_calibration_sum{0};
    double inclination_z_calibration_sum{0};
    double inclination_x_average{0};
    double inclination_y_average{0};
    double inclination_z_average{0};
  


    while (ros::ok())
    {
      switch (driver_stim300.update())
      {
        case Stim300Status::NORMAL:
          break;
        case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
          ROS_DEBUG("Stim 300 outside operating conditions");
        case Stim300Status::NEW_MEASURMENT:
              inclination_x = driver_stim300.getIncX();
              inclination_y = driver_stim300.getIncY();
              inclination_z = driver_stim300.getIncZ();
              Quaternion q;
              EulerAngles RPY;
           if (calibration_mode == true)
            {
              //std::cout<<"in calibration_mode"<<std::endl;
                if(number_of_samples < NUMBER_OF_CALIBRATION_SAMPLES)
                {
                    //std::cout<<"in calibration_mode"<<std::endl;
                    number_of_samples++;
                    //std::cout<<number_of_samples<<std::endl;
                    inclination_x_calibration_sum += inclination_x;
                    inclination_y_calibration_sum += inclination_y;
                    inclination_z_calibration_sum += inclination_z;

                }
                else
                {
                    //std::cout<<"in else"<<std::endl;
                    inclination_x_average = inclination_x_calibration_sum/NUMBER_OF_CALIBRATION_SAMPLES;
                    inclination_y_average = inclination_y_calibration_sum/NUMBER_OF_CALIBRATION_SAMPLES;
                    inclination_z_average = inclination_z_calibration_sum/NUMBER_OF_CALIBRATION_SAMPLES;

                    average_calibration_roll = atan2(inclination_y_average,inclination_z_average);
                    average_calibration_pitch = atan2(-inclination_x_average,sqrt(pow(inclination_y_average,2)+pow(inclination_z_average,2)));
                    std::cout<<average_calibration_roll<<std::endl;
                    std::cout<<average_calibration_pitch<<std::endl;
                    ROS_INFO("roll: %f", average_calibration_roll);
                    ROS_INFO("pitch: %f", average_calibration_pitch);
                    ROS_INFO("IMU Calibrated");
                    calibration_mode = false;
                }
              break;  
            }
            else
            {
                    RPY.roll = atan2(inclination_y,inclination_z);
                    RPY.pitch = atan2(-inclination_x,sqrt(pow(inclination_y,2)+pow(inclination_z,2)));
                    q = FromRPYToQuaternion(RPY);
                    stim300msg.header.stamp = ros::Time::now();
                    stim300msg.linear_acceleration.x = driver_stim300.getAccX() * gravity;
                    stim300msg.linear_acceleration.y = driver_stim300.getAccY() * gravity;
                    stim300msg.linear_acceleration.z = driver_stim300.getAccZ() * gravity;
                    stim300msg.angular_velocity.x = driver_stim300.getGyroX();
                    stim300msg.angular_velocity.y = driver_stim300.getGyroY();
                    stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
                    stim300msg.orientation.w = q.w;
                    stim300msg.orientation.x = q.x;
                    stim300msg.orientation.y = q.y;
                    stim300msg.orientation.z = q.z;
                    imuSensorPublisher.publish(stim300msg);
                    break;
            }
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