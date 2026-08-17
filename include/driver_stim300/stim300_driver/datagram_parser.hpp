#ifndef DRIVER_STIM300_DATAGRAM_PARSER_H
#define DRIVER_STIM300_DATAGRAM_PARSER_H

#include "stim300_constants.hpp"

#include <array>
#include <cstdint>

namespace stim_300 {

using namespace stim_const;

struct SensorConfig {
  char revision;
  uint8_t firmware_version;
  SampleFreq sample_freq;
  DatagramIdentifier datagram_id;
  bool normal_datagram_crlf;
  GyroOutputUnit gyro_output_unit;
  AccOutputUnit acc_output_unit;
  InclOutputUnit incl_output_unit;
  AccRange acc_range;

  bool operator!=(const SensorConfig &rhs) const {
    return sample_freq != rhs.sample_freq || datagram_id != rhs.datagram_id ||
           normal_datagram_crlf != rhs.normal_datagram_crlf ||
           gyro_output_unit != rhs.gyro_output_unit ||
           acc_output_unit != rhs.acc_output_unit ||
           incl_output_unit != rhs.incl_output_unit ||
           acc_range != rhs.acc_range;
  }
};

struct SensorData {
  std::array<double, 3> gyro{};
  std::array<double, 3> acc{};
  std::array<double, 3> incl{};

  std::array<double, 3> temp_gyro{};
  std::array<double, 3> temp_acc{};
  std::array<double, 3> temp_incl{};

  double aux{};
  uint8_t status{};
  uint8_t counter{};
  uint16_t latency_us{};
};

class DatagramParser {
public:
  DatagramParser(DatagramIdentifier dg_id, GyroOutputUnit gyro_o,
                 AccOutputUnit acc_o, InclOutputUnit incl_o,
                 AccRange acc_range);

  void set_data_parameters(const SensorConfig &sensor_config);

  [[nodiscard]]
  static uint32_t parse_crc(const uint8_t *buf);

  [[nodiscard]]
  SensorData parse_data(const uint8_t *buf) const;

  [[nodiscard]]
  SensorConfig parse_config(const uint8_t *cfg) const;

private:
  void set_data_scales(GyroOutputUnit gyro_o, AccOutputUnit acc_o,
                       InclOutputUnit incl_o, AccRange acc_range);

  std::array<bool, 5> is_included_{};
  double temp_scale_{};
  double aux_scale_{};
  double gyro_scale_{};
  double acc_scale_{};
  double incl_scale_{};
};

} // namespace stim_300

#endif // DRIVER_STIM300_DATAGRAM_PARSER_H
