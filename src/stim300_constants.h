#ifndef DRIVER_STIM300_STIM300_CONSTANTS_H
#define DRIVER_STIM300_STIM300_CONSTANTS_H

#include <cstdint>
#include <math.h>
#include <array>

namespace stim_const
{
static constexpr uint8_t N_BYTES_DATAGRAM_ID = 1;
static constexpr uint8_t N_BYTES_INERTIAL_SENSOR = 3;
static constexpr uint8_t N_BYTES_TEMP_SENSOR = 2;
static constexpr uint8_t N_BYTES_AUX_SENSOR = 3;
static constexpr uint8_t N_BYTES_COUNTER = 1;
static constexpr uint8_t N_BYTES_LATENCY = 2;
static constexpr uint8_t N_BYTES_CRC = 4;
static constexpr uint8_t N_BYTES_STATUS = 1;
static constexpr uint8_t N_BYTES_TERMINATION = 2;
static constexpr uint8_t MAX_DATAGRAM_SIZE = 65;

static constexpr double STIM300_GRAVITY = 9.81;

enum class DatagramIdentifier
{
  RATE,
  RATE_ACC,
  RATE_INCL,
  RATE_ACC_INCL,
  RATE_TEMP,
  RATE_ACC_TEMP,
  RATE_INCL_TEMP,
  RATE_ACC_INCL_TEMP,
  RATE_AUX,
  RATE_ACC_AUX,
  RATE_INCL_AUX,
  RATE_ACC_INCL_AUX,
  RATE_TEMP_AUX,
  RATE_ACC_TEMP_AUX,
  RATE_INCL_TEMP_AUX,
  RATE_ACC_INCL_TEMP_AUX,
  CONFIGURATION,
  CONFIGURATION_CRLF
};

enum class AccRange
{
  G2,
  G5,
  G10,
  G30,
  G80
};

enum class BaudRate
{  // defined as bit-rate in datasheet
  BAUD_377400,
  BAUD_460800,
  BAUD_921600,
  BAUD_1843200,
};

enum class GyroOutputUnit
{
  ANGULAR_RATE,
  AVERAGE_ANGULAR_RATE,
  INCREMENTAL_ANGLE,
  INTEGRATED_ANGLE
};
enum class AccOutputUnit
{
  ACCELERATION,
  AVERAGE_ACCELERATION,
  INCREMENTAL_VELOCITY,
  INTEGRATED_VELOCITY
};
enum class InclOutputUnit
{
  ACCELERATION,
  AVERAGE_ACCELERATION,
  INCREMENTAL_VELOCITY,
  INTEGRATED_VELOCITY
};

enum class SampleFreq
{
  S125,
  S250,
  S500,
  S1000,
  S2000,
  TRG
};

enum SensorIndx
{
  GYRO = 0,
  ACC,
  INCL,
  TEMP,
  AUX
};

struct DatagramInfo
{
  DatagramIdentifier id;
  uint8_t raw_id;
  std::array<bool, 5> included_sensors;
  uint8_t number_of_padding_bytes;
};

static constexpr std::array<DatagramInfo, 18> datagram_info_map  // rate,  acc, incl, temp,  aux
  { {
    { DatagramIdentifier::RATE, 0x90, { true, false, false, false, false }, 2 },
    { DatagramIdentifier::RATE_ACC, 0x91, { true, true, false, false, false }, 0 },
    { DatagramIdentifier::RATE_INCL, 0x92, { true, false, true, false, false }, 0 },
    { DatagramIdentifier::RATE_ACC_INCL, 0x93, { true, true, true, false, false }, 2 },
    { DatagramIdentifier::RATE_TEMP, 0x94, { true, false, false, true, false }, 3 },
    { DatagramIdentifier::RATE_ACC_TEMP, 0xA5, { true, true, false, true, false }, 2 },
    { DatagramIdentifier::RATE_INCL_TEMP, 0xA6, { true, false, true, true, false }, 2 },
    { DatagramIdentifier::RATE_ACC_INCL_TEMP, 0xA7, { true, true, true, true, false }, 1 },
    { DatagramIdentifier::RATE_AUX, 0x98, { true, false, false, false, true }, 2 },
    { DatagramIdentifier::RATE_ACC_AUX, 0x99, { true, true, false, false, true }, 0 },
    { DatagramIdentifier::RATE_INCL_AUX, 0x9A, { true, false, true, false, true }, 0 },
    { DatagramIdentifier::RATE_ACC_INCL_AUX, 0x9B, { true, true, true, false, true }, 2 },
    { DatagramIdentifier::RATE_TEMP_AUX, 0x9C, { true, false, false, true, true }, 3 },
    { DatagramIdentifier::RATE_ACC_TEMP_AUX, 0xAD, { true, true, false, true, true }, 2 },
    { DatagramIdentifier::RATE_INCL_TEMP_AUX, 0xAE, { true, false, true, true, true }, 2 },
    { DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX, 0xAF, { true, true, true, true, true }, 1 },
    { DatagramIdentifier::CONFIGURATION, 0xBC, { false, false, false, false, false }, 2 },
    { DatagramIdentifier::CONFIGURATION_CRLF, 0xBD, { false, false, false, false, false }, 2 },
  } };

constexpr uint8_t datagramIdentifierToRaw(DatagramIdentifier d_id)
{
  for (int i = 0; i < datagram_info_map.size(); ++i)
  {
    if (datagram_info_map[i].id == d_id)
    {
      return datagram_info_map[i].raw_id;
    }
  }
}

constexpr DatagramIdentifier rawToDatagramIdentifier(uint8_t datagram_id)
{
  for (int i = 0; i < datagram_info_map.size(); ++i)
  {
    if (datagram_info_map[i].raw_id == datagram_id)
    {
      return datagram_info_map[i].id;
    }
  }
  return DatagramIdentifier::CONFIGURATION_CRLF;  // Todo: implement error handeling
};

constexpr uint8_t numberOfPaddingBytes(DatagramIdentifier datagram_identifier)
{
  for (int i = 0; i < datagram_info_map.size(); ++i)
  {
    if (datagram_info_map[i].id == datagram_identifier)
    {
      return datagram_info_map[i].number_of_padding_bytes;
    }
  }
}

constexpr std::array<bool, 5> isIncluded(DatagramIdentifier datagram_identifier)
{
  for (int i = 0; i < datagram_info_map.size(); ++i)
  {
    if (datagram_info_map[i].id == datagram_identifier)
    {
      return datagram_info_map[i].included_sensors;
    }
  }
}

static DatagramIdentifier toDatagramID(std::array<bool, 5> isIncluded)
{
  for (int i = 0; i < datagram_info_map.size(); ++i)
  {
    if (datagram_info_map[i].included_sensors == isIncluded)
    {
      return datagram_info_map[i].id;
    }
  }
}

static const uint8_t calculateDatagramSize(DatagramIdentifier datagram_identifier)
{
  if (datagram_identifier == DatagramIdentifier::CONFIGURATION or
      datagram_identifier == DatagramIdentifier::CONFIGURATION_CRLF)
  {
    return 26;  // CR LF ending not included in datagram.
  }

  std::array<bool, 5> is_included = isIncluded(datagram_identifier);
  uint8_t n_inertial_sensors{ 1 };
  n_inertial_sensors += is_included[SensorIndx::ACC] ? 1 : 0;
  n_inertial_sensors += is_included[SensorIndx::INCL] ? 1 : 0;

  uint8_t size{ 0 };
  size += N_BYTES_DATAGRAM_ID;
  size += n_inertial_sensors * (3 * N_BYTES_INERTIAL_SENSOR + N_BYTES_STATUS);
  size += is_included[SensorIndx::TEMP] ? n_inertial_sensors * (3 * N_BYTES_TEMP_SENSOR + N_BYTES_STATUS) : 0;
  size += is_included[SensorIndx::AUX] ? N_BYTES_AUX_SENSOR + N_BYTES_STATUS : 0;
  size += N_BYTES_COUNTER;
  size += N_BYTES_LATENCY;
  size += N_BYTES_CRC;

  return size;
}

constexpr uint32_t powerOf2(uint8_t power)
{
  return 1 << power;
}

static constexpr double accScale(AccRange acc_range)
{
  switch (acc_range)
  {
    case AccRange::G2:
      return (STIM300_GRAVITY) / powerOf2(21);
    case AccRange::G5:
      return (STIM300_GRAVITY) / powerOf2(20);
    case AccRange::G10:
      return (STIM300_GRAVITY) / powerOf2(19);
    case AccRange::G30:
      return (STIM300_GRAVITY) / powerOf2(18);
    case AccRange::G80:
      return (STIM300_GRAVITY) / powerOf2(16);
  }
}

static constexpr double accIncrScale(AccRange acc_range)
{
  switch (acc_range)
  {
    case AccRange::G2:
      return 1.0 / powerOf2(24);
    case AccRange::G5:
      return 1.0 / powerOf2(23);
    case AccRange::G10:
      return 1.0 / powerOf2(22);
    case AccRange::G30:
      return 1.0 / powerOf2(21);
    case AccRange::G80:
      return 1.0 / powerOf2(19);
  }
}

static constexpr double gyroIncrScale()
{
  return (M_PI / 180.00) / powerOf2(21);
}
static constexpr double gyroScale()
{
  return (M_PI / 180.00) / powerOf2(14);
}
static constexpr double inclScale()
{
  return (STIM300_GRAVITY) / powerOf2(22);
}
static constexpr double inclIncrScale()
{
  return 1.0 / powerOf2(25);
}
static constexpr double tempScale()
{
  return 1.0 / powerOf2(8);
}
static constexpr double auxScale()
{
  return 5.0 / powerOf2(24);
}

}  // namespace stim_const
#endif  // DRIVER_STIM300_STIM300_CONSTANTS_H
