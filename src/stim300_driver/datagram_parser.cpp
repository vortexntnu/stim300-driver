#include "stim300_driver/datagram_parser.hpp"
#include <cstdint>

using namespace stim_const;
using namespace stim_300;

namespace {

constexpr uint8_t SAMPLE_FREQ_SHIFT = 5;
constexpr uint8_t OUTPUT_UNIT_MASK = 0x0F;
constexpr uint8_t ACC_RANGE_SHIFT = 4;

constexpr uint8_t CRLF_BIT = 0;
constexpr uint8_t ACC_BIT = 1;
constexpr uint8_t INCL_BIT = 2;
constexpr uint8_t TEMP_BIT = 3;
constexpr uint8_t AUX_BIT = 4;

constexpr bool bit_is_set(uint8_t value, uint8_t bit) {
  return (value & (1u << bit)) != 0;
}

constexpr uint8_t sample_freq_code(uint8_t value) {
  return value >> SAMPLE_FREQ_SHIFT;
}

constexpr uint8_t output_unit_code(uint8_t value) {
  return value & OUTPUT_UNIT_MASK;
}

constexpr uint8_t acc_range_code(uint8_t value) {
  return value >> ACC_RANGE_SHIFT;
}

} // namespace

uint16_t read_u16_be(const uint8_t *buf) {
  return (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]);
}

uint32_t read_u24_be(const uint8_t *buf) {
  return (static_cast<uint32_t>(buf[0]) << 16) |
         (static_cast<uint32_t>(buf[1]) << 8) | static_cast<uint32_t>(buf[2]);
}

uint32_t read_u32_be(const uint8_t *buf) {
  return (static_cast<uint32_t>(buf[0]) << 24) |
         (static_cast<uint32_t>(buf[1]) << 16) |
         (static_cast<uint32_t>(buf[2]) << 8) | static_cast<uint32_t>(buf[3]);
}
int16_t read_i16_be(const uint8_t *buf) {
  uint16_t value =
      (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]);

  return static_cast<int16_t>(value);
}

int32_t read_i24_be(const uint8_t *buf) {
  uint32_t value = (static_cast<uint32_t>(buf[0]) << 16) |
                   (static_cast<uint32_t>(buf[1]) << 8) |
                   static_cast<uint32_t>(buf[2]);

  if (value & 0x00800000)
    value |= 0xFF000000;

  return static_cast<int32_t>(value);
}

DatagramParser::DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o,
                               AccOutputUnit acc_o, InclOutputUnit incl_o,
                               AccRange acc_range)
    : is_included_(isIncluded(dg_id)), temp_scale_(tempScale()),
      aux_scale_(auxScale()) {
  setDataScales(gyro_o, acc_o, incl_o, acc_range);
}
void DatagramParser::setDataParameters(SensorConfig sensor_config) {
  is_included_ = isIncluded(sensor_config.datagram_id);
  setDataScales(sensor_config.gyro_output_unit, sensor_config.acc_output_unit,
                sensor_config.incl_output_unit, sensor_config.acc_range);
}
void DatagramParser::setDataScales(GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                                   InclOutputUnit incl_o, AccRange acc_range) {
  switch (gyro_o) {
  case GyroOutputUnit::ANGULAR_RATE:         // units are in rad/s
  case GyroOutputUnit::AVERAGE_ANGULAR_RATE: // units are in rad/s
    gyro_scale_ = gyroScale();
    break;
  case GyroOutputUnit::INCREMENTAL_ANGLE: // units are in rad/sample
  case GyroOutputUnit::INTEGRATED_ANGLE:  // units are in rad
    gyro_scale_ = gyroIncrScale();
    break;
  }
  switch (acc_o) {
  case AccOutputUnit::ACCELERATION:         // units are in g
  case AccOutputUnit::AVERAGE_ACCELERATION: // units are in g
    acc_scale_ = accScale(acc_range);
    break;
  case AccOutputUnit::INCREMENTAL_VELOCITY: // units are in m/s/sample
  case AccOutputUnit::INTEGRATED_VELOCITY:
    acc_scale_ = accIncrScale(acc_range);
    break;
  }
  switch (incl_o) {
  case InclOutputUnit::ACCELERATION:         // units are in g
  case InclOutputUnit::AVERAGE_ACCELERATION: // units are in g
    incl_scale_ = inclScale();
    break;
  case InclOutputUnit::INCREMENTAL_VELOCITY: // units are in m/s/sample
  case InclOutputUnit::INTEGRATED_VELOCITY:
    incl_scale_ = inclIncrScale();
    break;
  }
}

uint32_t DatagramParser::parse_crc(const uint8_t *buf) {
  return parse_u32_be(buf);
}

[[nodiscard]]
SensorData DatagramParser::parse_data(const uint8_t *buf) const {
  SensorData data{};
  const uint8_t *it = buf + N_BYTES_DATAGRAM_ID;

  auto read_vec_i24 = [&](auto &dst, auto scale) {
    for (auto &value : dst) {
      value = scale * read_i24_be(it);
      it += N_BYTES_INERTIAL_SENSOR;
    }

    data.status |= *it;
    it += N_BYTES_STATUS;
  };

  auto read_vec_i16 = [&](auto &dst, auto scale) {
    for (auto &value : dst) {
      value = scale * read_i16_be(it);
      it += N_BYTES_TEMP_SENSOR;
    }

    data.status |= *it;
    it += N_BYTES_STATUS;
  };

  read_vec_i24(data.gyro, gyro_scale_);
  read_vec_i24(data.acc, acc_scale_);
  read_vec_i24(data.incl, incl_scale_);

  read_vec_i16(data.temp_gyro, temp_scale_);
  read_vec_i16(data.temp_acc, temp_scale_);
  read_vec_i16(data.temp_incl, temp_scale_);

  data.aux = aux_scale_ * read_i24_be(it);
  it += N_BYTES_AUX_SENSOR;

  data.status |= *it;
  it += N_BYTES_STATUS;

  data.counter = *it;
  it += N_BYTES_COUNTER;

  data.latency_us = read_u16_be(it);
  it += N_BYTES_LATENCY;

  return data;
}

SensorConfig DatagramParser::parse_config(const uint8_t *cfg) const {
  SensorConfig sensor_config{};

  sensor_config.revision = cfg[1];
  sensor_config.firmvare_version = cfg[2];

  const uint8_t datagram_config = cfg[3];

  switch (sample_freq_code(datagram_config)) {
  case 0:
    sensor_config.sample_freq = SampleFreq::S125;
    break;
  case 1:
    sensor_config.sample_freq = SampleFreq::S250;
    break;
  case 2:
    sensor_config.sample_freq = SampleFreq::S500;
    break;
  case 3:
    sensor_config.sample_freq = SampleFreq::S1000;
    break;
  case 4:
    sensor_config.sample_freq = SampleFreq::S2000;
    break;
  case 5:
    sensor_config.sample_freq = SampleFreq::TRG;
    break;
  }

  sensor_config.normal_datagram_CRLF = bit_is_set(datagram_config, CRLF_BIT);

  std::array<bool, 5> included_sensors{};

  included_sensors[SensorIndx::GYRO] = true;
  included_sensors[SensorIndx::ACC] = bit_is_set(datagram_config, ACC_BIT);
  included_sensors[SensorIndx::INCL] = bit_is_set(datagram_config, INCL_BIT);
  included_sensors[SensorIndx::TEMP] = bit_is_set(datagram_config, TEMP_BIT);
  included_sensors[SensorIndx::AUX] = bit_is_set(datagram_config, AUX_BIT);

  sensor_config.datagram_id = toDatagramID(included_sensors);

  const uint8_t gyro_unit = output_unit_code(cfg[5]);

  switch (gyro_unit) {
  case 0:
  case 4:
    sensor_config.gyro_output_unit = GyroOutputUnit::ANGULAR_RATE;
    break;

  case 1:
  case 5:
    sensor_config.gyro_output_unit = GyroOutputUnit::INCREMENTAL_ANGLE;
    break;

  case 2:
  case 6:
    sensor_config.gyro_output_unit = GyroOutputUnit::AVERAGE_ANGULAR_RATE;
    break;

  case 3:
  case 7:
    sensor_config.gyro_output_unit = GyroOutputUnit::INTEGRATED_ANGLE;
    break;
  }

  const uint8_t acc_unit = output_unit_code(cfg[8]);

  switch (acc_unit) {
  case 0:
    sensor_config.acc_output_unit = AccOutputUnit::ACCELERATION;
    break;
  case 1:
    sensor_config.acc_output_unit = AccOutputUnit::INCREMENTAL_VELOCITY;
    break;
  case 2:
    sensor_config.acc_output_unit = AccOutputUnit::AVERAGE_ACCELERATION;
    break;
  case 3:
    sensor_config.acc_output_unit = AccOutputUnit::INTEGRATED_VELOCITY;
    break;
  }

  const uint8_t incl_unit = output_unit_code(cfg[11]);

  switch (incl_unit) {
  case 0:
    sensor_config.incl_output_unit = InclOutputUnit::ACCELERATION;
    break;
  case 1:
    sensor_config.incl_output_unit = InclOutputUnit::INCREMENTAL_VELOCITY;
    break;
  case 2:
    sensor_config.incl_output_unit = InclOutputUnit::AVERAGE_ACCELERATION;
    break;
  case 3:
    sensor_config.incl_output_unit = InclOutputUnit::INTEGRATED_VELOCITY;
    break;
  }

  const uint8_t range = acc_range_code(cfg[17]);

  switch (range) {
  case 0:
    sensor_config.acc_range = AccRange::G10;
    break;
  case 2:
    sensor_config.acc_range = AccRange::G2;
    break;
  case 3:
    sensor_config.acc_range = AccRange::G5;
    break;
  case 4:
    sensor_config.acc_range = AccRange::G30;
    break;
  case 6:
    sensor_config.acc_range = AccRange::G80;
    break;
  }

  return sensor_config;
}
