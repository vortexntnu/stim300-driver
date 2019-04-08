
#include "datagram_parser.h"

using namespace stim_300;

DatagramParser::DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                               InclOutputUnit incl_o)
  : is_included_(isIncluded(dg_id))
  , gyro_scale_([=] {
    switch (gyro_o)
    {
      case GyroOutputUnit::ANGULAR_RATE:          // units are in rad/s
      case GyroOutputUnit::AVERAGE_ANGULAR_RATE:  // units are in rad/s
        return GYRO_SCALE;
      case GyroOutputUnit::INCREMENTAL_ANGLE:  // units are in rad/sample
      case GyroOutputUnit::INTEGRATED_ANGLE:   // units are in rad
        return GYRO_INCR_SCALE;
    }
  }())
  , acc_scale_([=] {
    switch (acc_o)
    {
      case AccOutputUnit::ACCELERATION:          // units are in m/s^2
      case AccOutputUnit::AVERAGE_ACCELERATION:  // units are in m/s^2
        return ACC_SCALE;
      case AccOutputUnit::INCREMENTAL_VELOCITY:  // units are in m/s/sample
        return ACC_INCR_SCALE;
    }
  }())
  , incl_scale_([=] {
    switch (incl_o)
    {
      case InclOutputUnit::ACCELERATION:          // units are in m/s^2
      case InclOutputUnit::AVERAGE_ACCELERATION:  // units are in m/s^2
        return INCL_SCALE;
      case InclOutputUnit::INCREMENTAL_VELOCITY:  // units are in m/s/sample
        return INCL_INCR_SCALE;
    }
  }())
{
}


bool DatagramParser::parseDatagram(std::vector<uint8_t>::const_iterator& buffer_itr, SensorData& sensor_data) const
{
  uint8_t status{ 0 };

  if (is_included_[SensorIndx::GYRO])
  {
    for (auto& gyro : sensor_data.gyro)
      gyro = gyro_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::ACC])
  {
    for (auto& acc : sensor_data.acc)
      acc = acc_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::INCL])
  {
    for (auto& incl : sensor_data.incl)
      incl = incl_scale_ * parseTwosComplement(buffer_itr, N_BYTES_INERTIAL_SENSOR);
    status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  if (is_included_[SensorIndx::TEMP])
  {
    if (is_included_[SensorIndx::GYRO])
    {
      for (auto& temp : sensor_data.temp_gyro)
        temp = TEMP_SCALE * parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
      status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
    if (is_included_[SensorIndx::ACC])
    {
      for (auto& temp : sensor_data.temp_acc)
        temp = TEMP_SCALE * parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
      status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
    if (is_included_[SensorIndx::INCL])
    {
      for (auto& temp : sensor_data.temp_incl)
        temp = TEMP_SCALE * parseTwosComplement(buffer_itr, N_BYTES_TEMP_SENSOR);
      status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
    }
  }
  if (is_included_[SensorIndx::AUX])
  {
    sensor_data.aux = AUX_SCALE * parseTwosComplement(buffer_itr, N_BYTES_AUX_SENSOR);
    status += parseUnsigned(buffer_itr, N_BYTES_STATUS);
  }
  sensor_data.counter = parseUnsigned(buffer_itr, N_BYTES_COUNTER);

  sensor_data.latency_us = parseUnsigned(buffer_itr, N_BYTES_LATENCY);

  sensor_data.crc = parseUnsigned(buffer_itr, N_BYTES_CRC);

  return status == 0;
}

