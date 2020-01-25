
#ifndef DRIVER_STIM300_DATAGRAM_PARSER_H
#define DRIVER_STIM300_DATAGRAM_PARSER_H

#include <vector>
#include <cstdint>
#include <array>
#include "stim300_constants.h"
#include <sstream>
#include <cassert>
using namespace stim_const;

namespace stim_300
{
struct SensorConfig
{
  char revision;
  uint8_t firmvare_version;
  SampleFreq sample_freq;
  DatagramIdentifier datagram_id;
  bool normal_datagram_CRLF;
  GyroOutputUnit gyro_output_unit;
  AccOutputUnit acc_output_unit;
  InclOutputUnit incl_output_unit;
  AccRange acc_range;

  inline bool operator!=(const SensorConfig& rhs)
  {
    return this->sample_freq != rhs.sample_freq or this->datagram_id != rhs.datagram_id or
           this->normal_datagram_CRLF != rhs.normal_datagram_CRLF or this->gyro_output_unit != rhs.gyro_output_unit or
           this->acc_output_unit != rhs.acc_output_unit or this->incl_output_unit != rhs.incl_output_unit or
           this->acc_range != rhs.acc_range;
  }

  std::string print() const
  {
    std::stringstream ss;
    ss << "Firmware: " << revision << std::to_string(firmvare_version) << std::endl;
    ss << "Sample_freq: ";
    switch (sample_freq)
    {
      case SampleFreq::S125:
        ss << "125 Hz";
        break;
      case SampleFreq::S250:
        ss << "250 Hz";
        break;
      case SampleFreq::S500:
        ss << "500 Hz";
        break;
      case SampleFreq::S1000:
        ss << "1000 Hz";
        break;
      case SampleFreq::S2000:
        ss << "2000 Hz";
        break;
      case SampleFreq::TRG:
        ss << "External Trigger";
        break;
    }
    ss << std::endl;
    auto included_sensors = isIncluded(datagram_id);
    ss << "Gyro:\t\t\t" << included_sensors[SensorIndx::GYRO] << std::endl;
    ss << "Accelerometer:\t" << included_sensors[SensorIndx::ACC] << std::endl;
    ss << "Inlcinometer:\t" << included_sensors[SensorIndx::INCL] << std::endl;
    ss << "Temprature:\t\t" << included_sensors[SensorIndx::TEMP] << std::endl;
    ss << "Aux:\t\t\t" << included_sensors[SensorIndx::AUX] << std::endl;
    ss << "Normal Datagram termination: " << normal_datagram_CRLF << std::endl;
    ss << "Gyro output:\t\t\t";
    switch (gyro_output_unit)
    {
      case GyroOutputUnit::ANGULAR_RATE:
        ss << "Angular rate";
        break;
      case GyroOutputUnit::AVERAGE_ANGULAR_RATE:
        ss << "Average angular rate";
        break;
      case GyroOutputUnit::INCREMENTAL_ANGLE:
        ss << "Incremental angle";
        break;
      case GyroOutputUnit::INTEGRATED_ANGLE:
        ss << "Integrated angle";
        break;
    }
    ss << std::endl;
    ss << "Accelerometer output:\t";
    switch (acc_output_unit)
    {
      case AccOutputUnit::ACCELERATION:
        ss << "Acceleration";
        break;
      case AccOutputUnit::AVERAGE_ACCELERATION:
        ss << "Average acceleration";
        break;
      case AccOutputUnit::INCREMENTAL_VELOCITY:
        ss << "Incremental velocity";
        break;
      case AccOutputUnit::INTEGRATED_VELOCITY:
        ss << "Integrated velocity";
        break;
    }
    ss << std::endl;
    ss << "Inclinometer output:\t";
    switch (incl_output_unit)
    {
      case InclOutputUnit::ACCELERATION:
        ss << "Acceleration";
        break;
      case InclOutputUnit::AVERAGE_ACCELERATION:
        ss << "Average acceleration";
        break;
      case InclOutputUnit::INCREMENTAL_VELOCITY:
        ss << "Incremental velocity";
        break;
      case InclOutputUnit::INTEGRATED_VELOCITY:
        ss << "Integrated velocity";
        break;
    }
    ss << std::endl;
    ss << "Acceleration range: ";
    switch (acc_range)
    {
      case AccRange::G2:
        ss << "2";
        break;
      case AccRange::G5:
        ss << "5";
        break;
      case AccRange::G10:
        ss << "10";
        break;
      case AccRange::G30:
        ss << "30";
        break;
      case AccRange::G80:
        ss << "80";
        break;
    }
    ss << " g." << std::endl;
    return ss.str();
  }
};

struct SensorData
{
  std::array<double, 3> gyro;
  std::array<double, 3> acc;
  std::array<double, 3> incl;
  std::array<double, 3> temp_gyro;
  std::array<double, 3> temp_acc;
  std::array<double, 3> temp_incl;
  double aux;
  uint8_t counter;
  uint16_t latency_us;
};

struct DatagramParser
{
  DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o, InclOutputUnit incl_o,
                 AccRange acc_range);
  void setDataParameters(SensorConfig sensor_config);
  static uint32_t parseCRC(std::vector<uint8_t>::const_iterator&& itr);
  uint8_t parseData(std::vector<uint8_t>::const_iterator&& buffer_itr, SensorData& sensor_data) const;
  SensorConfig parseConfig(std::vector<uint8_t>::const_iterator&& buffer_itr) const;

private:
  std::array<bool, 5> is_included_;
  double temp_scale_;
  double aux_scale_;
  double gyro_scale_;
  double acc_scale_;
  double incl_scale_;
  void setDataScales(GyroOutputUnit gyro_o, AccOutputUnit acc_o, InclOutputUnit incl_o, AccRange acc_range);
  // Meta data is stored as "unsigned word", we simply combine the bytes into the right
  // sized uint by left shifting them. Note the biggest is the CRC which is 32 bits.
  static const uint32_t parseUnsigned(std::vector<uint8_t>::const_iterator& it, uint8_t size)
  {
    uint32_t tmp{ 0 };
    for (int i = 0; i < size; ++i)
    {
      tmp = (tmp << 8u) | *(it++);
    }
    return tmp;
  }

  // Sensor data is stored as two`s complement, we shift the bytes according to the datasheet,
  // bu we shift them to fill up the int32_t type then we shift them back to their right
  // position. When we shift them back the shift operator will automatically sign-extend the
  // value.
  static const int32_t parseTwosComplement(std::vector<uint8_t>::const_iterator& it, const uint8_t size)
  {
    assert(size == 3 or size == 2);
    if (size == 3)
    {
      return ((*it++ << 24) | (*it++ << 16) | (*it++ << 8)) >> 8;
    }
    else if (size == 2)
    {
      return ((*it++ << 24) | (*it++ << 16)) >> 16;
    }
  }
};
}  // end namespace stim_300

#endif  // DRIVER_STIM300_DATAGRAM_PARSER_H
