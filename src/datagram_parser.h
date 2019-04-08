
#ifndef DRIVER_STIM300_DATAGRAM_PARSER_H
#define DRIVER_STIM300_DATAGRAM_PARSER_H

#include <vector>
#include <cstdint>
#include <array>
#include "stim300_constants.h"

namespace stim_300
{
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
  uint32_t crc;
};

struct DatagramParser
{
  DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o, AccOutputUnit acc_o, InclOutputUnit incl_o);
  uint8_t getDatagramSize() const;
  bool parseDatagram(std::vector<uint8_t>::const_iterator& buffer_itr, SensorData& sensor_data) const;

private:
  enum SensorIndx
  {
    GYRO = 0,
    ACC,
    INCL,
    TEMP,
    AUX
  };
  double gyro_scale_;
  double acc_scale_;
  double incl_scale_;

  constexpr std::array<bool, 5> isIncluded(DatagramIdentifier datagram_identifier) const;
  std::array<bool, 5> is_included_;
  bool use_termination_;

  // Meta data is stored as "unsigned word", we simply combine the bytes into the right
  // sized uint by left shifting them. Note the biggest is the CRC which is 32 bits.
  static constexpr uint32_t parseUnsigned(std::vector<uint8_t>::const_iterator& it, uint8_t size)
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
  static constexpr int32_t parseTwosComplement(std::vector<uint8_t>::const_iterator& it, uint8_t size)
  {
    if (size == 3)
    {
      return ((*it++ << 24) | (*it++ << 16) | (*it++ << 8)) >> 8;
    }
    else if (size == 2)
    {
      return ((*it++ << 24) | (*it++ << 16)) >> 16;
    }
    else
    {
      throw std::runtime_error("Unsuported input size for parseTwoComplement");
    }
  }
};
}  // end namespace stim_300

#endif  // DRIVER_STIM300_DATAGRAM_PARSER_H
