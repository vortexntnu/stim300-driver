#ifndef DRIVER_STIM300_STIM300_CONSTANTS_H
#define DRIVER_STIM300_STIM300_CONSTANTS_H

#include <cstdint>
#include <math.h>

namespace stim_300
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
static constexpr double TWO_POWER_14 = 16'384;      // 2^14
static constexpr double TWO_POWER_20 = 1'048'576;   // 2^20
static constexpr double TWO_POWER_21 = 2'097'152;   // 2^21
static constexpr double TWO_POWER_22 = 4'194'304;   // 2^22
static constexpr double TWO_POWER_24 = 16'777'216;  // 2^24
static constexpr double TWO_POWER_25 = 33'554'432;  // 2^25
static constexpr double GYRO_INCR_SCALE = (M_PI / 180.00) / TWO_POWER_21;
static constexpr double GYRO_SCALE = (M_PI / 180.00) / TWO_POWER_14;
static constexpr double ACC_SCALE = (STIM300_GRAVITY) / TWO_POWER_20;
static constexpr double ACC_INCR_SCALE = 1 / TWO_POWER_22;
static constexpr double INCL_SCALE = (STIM300_GRAVITY) / TWO_POWER_22;
static constexpr double INCL_INCR_SCALE = 1 / TWO_POWER_25;
static constexpr double TEMP_SCALE = 1 / 256.00;
static constexpr double AUX_SCALE = 5.0 / TWO_POWER_24;

enum class DatagramIdentifier : uint8_t
{
  RATE = 0x90,
  RATE_ACC = 0x91,
  RATE_INCL = 0x92,
  RATE_ACC_INCL = 0x93,
  RATE_TEMP = 0x94,
  RATE_ACC_TEMP = 0xA5,
  RATE_INCL_TEMP = 0xA6,
  RATE_ACC_INCL_TEMP = 0xA7,
  RATE_AUX = 0x98,
  RATE_ACC_AUX = 0x99,
  RATE_INCL_AUX = 0x9A,
  RATE_ACC_INCL_AUX = 0x9B,
  RATE_TEMP_AUX = 0x9C,
  RATE_ACC_TEMP_AUX = 0xAD,
  RATE_INCL_TEMP_AUX = 0xAE,
  RATE_ACC_INCL_TEMP_AUX = 0xAF,
  CONFIGURATION = 0xBC,
  CONFIGURATION_CRLF = 0xBD
};

enum class BaudRate : uint8_t
{  // defined as bit-rate in datasheet
  BAUD_377400 = 0,
  BAUD_460800 = 1,
  BAUD_921600 = 2,
  BAUD_1843200 = 3,
};

enum class GyroOutputUnit : uint8_t
{
  ANGULAR_RATE,
  AVERAGE_ANGULAR_RATE,
  INCREMENTAL_ANGLE,
  INTEGRATED_ANGLE
};
enum class AccOutputUnit : uint8_t
{
  ACCELERATION,
  AVERAGE_ACCELERATION,
  INCREMENTAL_VELOCITY
};
enum class InclOutputUnit : uint8_t
{
  ACCELERATION,
  AVERAGE_ACCELERATION,
  INCREMENTAL_VELOCITY
};

constexpr uint8_t datagramIdentifierToRaw(DatagramIdentifier d_id)
{
  return static_cast<uint8_t>(d_id);
}

constexpr DatagramIdentifier rawToDatagramIdentifier(uint8_t datagram_id)
{
  switch (datagram_id)
  {
    case 0x90:
      return DatagramIdentifier::RATE;
    case 0x91:
      return DatagramIdentifier::RATE_ACC;
    case 0x92:
      return DatagramIdentifier::RATE_INCL;
    case 0x93:
      return DatagramIdentifier::RATE_ACC_INCL;
    case 0x94:
      return DatagramIdentifier::RATE_TEMP;
    case 0xA5:
      return DatagramIdentifier::RATE_ACC_TEMP;
    case 0xA6:
      return DatagramIdentifier::RATE_INCL_TEMP;
    case 0xA7:
      return DatagramIdentifier::RATE_ACC_INCL_TEMP;
    case 0x98:
      return DatagramIdentifier::RATE_AUX;
    case 0x99:
      return DatagramIdentifier::RATE_ACC_AUX;
    case 0x9A:
      return DatagramIdentifier::RATE_INCL_AUX;
    case 0x9B:
      return DatagramIdentifier::RATE_ACC_INCL_AUX;
    case 0x9C:
      return DatagramIdentifier::RATE_TEMP_AUX;
    case 0xAD:
      return DatagramIdentifier::RATE_ACC_TEMP_AUX;
    case 0xAE:
      return DatagramIdentifier::RATE_INCL_TEMP_AUX;
    case 0xAF:
      return DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX;
    case 0xBC:
      return DatagramIdentifier::CONFIGURATION;
    case 0xBD:
      return DatagramIdentifier::CONFIGURATION_CRLF;
    default:
      throw std::out_of_range("Raw value does not match any implemented datgram ids");
  }
};

constexpr uint8_t numberOfPaddingBytes(DatagramIdentifier datagram_identifier)
{
  switch (datagram_identifier)
  {
    case DatagramIdentifier::CONFIGURATION:
      return 2;
    case DatagramIdentifier::CONFIGURATION_CRLF:
      return 2;
    case DatagramIdentifier::RATE:
      return 2;
    case DatagramIdentifier::RATE_ACC:
      return 0;
    case DatagramIdentifier::RATE_INCL:
      return 0;
    case DatagramIdentifier::RATE_ACC_INCL:
      return 2;
    case DatagramIdentifier::RATE_TEMP:
      return 3;
    case DatagramIdentifier::RATE_ACC_TEMP:
      return 2;
    case DatagramIdentifier::RATE_INCL_TEMP:
      return 2;
    case DatagramIdentifier::RATE_ACC_INCL_TEMP:
      return 1;
    case DatagramIdentifier::RATE_AUX:
      return 2;
    case DatagramIdentifier::RATE_ACC_AUX:
      return 0;
    case DatagramIdentifier::RATE_INCL_AUX:
      return 0;
    case DatagramIdentifier::RATE_ACC_INCL_AUX:
      return 2;
    case DatagramIdentifier::RATE_TEMP_AUX:
      return 3;
    case DatagramIdentifier::RATE_ACC_TEMP_AUX:
      return 2;
    case DatagramIdentifier::RATE_INCL_TEMP_AUX:
      return 2;
    case DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX:
      return 1;
  }
}

}  // namespace stim_300
#endif  // DRIVER_STIM300_STIM300_CONSTANTS_H
