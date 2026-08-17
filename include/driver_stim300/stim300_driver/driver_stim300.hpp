#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include "datagram_parser.hpp"
#include "stim300_constants.hpp"

#include <array>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

enum class Stim300Status {
  NORMAL,
  NEW_MEASUREMENT,
  CONFIG_CHANGED,
  STARTING_SENSOR,
  SYSTEM_INTEGRITY_ERROR,
  OUTSIDE_OPERATING_CONDITIONS,
  OVERLOAD,
  ERROR_IN_MEASUREMENT_CHANNEL,
  ERROR
};

class DriverStim300 {
public:
  using StatusHandler = std::function<void(Stim300Status)>;

  DriverStim300(boost::asio::io_context &io_context, const std::string &device,
                stim_const::DatagramIdentifier datagram_id,
                stim_const::GyroOutputUnit gyro_output_unit,
                stim_const::AccOutputUnit acc_output_unit,
                stim_const::InclOutputUnit incl_output_unit,
                stim_const::AccRange acc_range, stim_const::SampleFreq freq);
  DriverStim300(boost::asio::io_context &io_context, const std::string &device);
  ~DriverStim300();

  DriverStim300(const DriverStim300 &) = delete;
  DriverStim300 &operator=(const DriverStim300 &) = delete;
  DriverStim300(DriverStim300 &&) = delete;
  DriverStim300 &operator=(DriverStim300 &&) = delete;

  void start_async_read(StatusHandler status_handler);
  void stop();

  double get_acc_x() const noexcept;
  double get_acc_y() const noexcept;
  double get_acc_z() const noexcept;
  double get_gyro_x() const noexcept;
  double get_gyro_y() const noexcept;
  double get_gyro_z() const noexcept;
  double get_inc_x() const noexcept;
  double get_inc_y() const noexcept;
  double get_inc_z() const noexcept;
  uint16_t get_sample_rate() const noexcept;
  uint16_t get_latency_us() const noexcept;
  double get_average_temp() const noexcept;
  std::string print_sensor_config() const noexcept;
  bool is_sensor_status_good() const noexcept;
  uint8_t get_internal_measurement_counter() const noexcept;

private:
  void async_read();
  void read_data_stream(const boost::system::error_code &error,
                        std::size_t bytes_read);
  void process_pending_data();
  Stim300Status parse_datagram();
  void set_datagram_format(stim_const::DatagramIdentifier id);
  static bool verify_checksum(const uint8_t *begin, std::size_t size,
                              uint8_t crc_dummy_bytes);
  void ask_for_config_datagram();

  boost::asio::serial_port serial_port_;
  std::array<uint8_t, 512> read_buffer_{};
  StatusHandler status_handler_;
  stim_300::DatagramParser datagram_parser_;
  std::vector<uint8_t> pending_bytes_{};

  uint8_t datagram_id_;
  uint8_t crc_dummy_bytes_;
  uint8_t datagram_size_;
  stim_300::SensorConfig sensor_config_;
  stim_300::SensorData sensor_data_{};
  uint8_t sensor_status_{0};
};

#endif
