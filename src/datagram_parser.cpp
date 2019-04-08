
#include "datagram_parser.h"

using namespace stim_300;

DatagramParser::DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                               InclOutputUnit incl_o)
  : is_included_(isIncluded(dg_id))
  , use_termination_{ false }
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

uint8_t DatagramParser::getDatagramSize() const
{
  uint8_t n_inertial_sensors{ 1 };
  n_inertial_sensors += is_included_[SensorIndx::ACC] ? 1 : 0;
  n_inertial_sensors += is_included_[SensorIndx::INCL] ? 1 : 0;

  uint8_t size{ 0 };
  size += N_BYTES_DATAGRAM_ID;
  size += n_inertial_sensors * (3 * N_BYTES_INERTIAL_SENSOR + N_BYTES_STATUS);
  size += is_included_[SensorIndx::TEMP] ? n_inertial_sensors * (3 * N_BYTES_TEMP_SENSOR + N_BYTES_STATUS) : 0;
  size += is_included_[SensorIndx::AUX] ? N_BYTES_AUX_SENSOR + N_BYTES_STATUS : 0;
  size += N_BYTES_COUNTER;
  size += N_BYTES_LATENCY;
  size += N_BYTES_CRC;
  size += use_termination_ ? N_BYTES_TERMINATION : 0;

  return size;
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

constexpr std::array<bool, 5> DatagramParser::isIncluded(DatagramIdentifier datagram_identifier) const
{
  switch (datagram_identifier)
  {
    // rate,  acc, incl, temp,  aux
    case DatagramIdentifier::RATE:
      return { true, false, false, false, false };
    case DatagramIdentifier::RATE_ACC:
      return { true, true, false, false, false };
    case DatagramIdentifier::RATE_INCL:
      return { true, false, true, false, false };
    case DatagramIdentifier::RATE_ACC_INCL:
      return { true, true, true, false, false };
    case DatagramIdentifier::RATE_TEMP:
      return { true, false, false, true, false };
    case DatagramIdentifier::RATE_ACC_TEMP:
      return { true, true, false, true, false };
    case DatagramIdentifier::RATE_INCL_TEMP:
      return { true, false, true, true, false };
    case DatagramIdentifier::RATE_ACC_INCL_TEMP:
      return { true, true, true, true, false };
    case DatagramIdentifier::RATE_AUX:
      return { true, false, false, false, true };
    case DatagramIdentifier::RATE_ACC_AUX:
      return { true, true, false, false, true };
    case DatagramIdentifier::RATE_INCL_AUX:
      return { true, false, true, false, true };
    case DatagramIdentifier::RATE_ACC_INCL_AUX:
      return { true, true, true, false, true };
    case DatagramIdentifier::RATE_TEMP_AUX:
      return { true, false, false, true, true };
    case DatagramIdentifier::RATE_ACC_TEMP_AUX:
      return { true, true, false, true, true };
    case DatagramIdentifier::RATE_INCL_TEMP_AUX:
      return { true, false, true, true, true };
    case DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX:
      return { true, true, true, true, true };
    case DatagramIdentifier::CONFIGURATION:
      return { false, false, false, false, false };
    case DatagramIdentifier::CONFIGURATION_CRLF:
      return { false, false, false, false, false };
    default:
      throw std::out_of_range("Undefined datagram identifier");
  }
}
