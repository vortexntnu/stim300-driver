#include "stim300_driver/driver_stim300.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <termios.h>

using namespace stim_const;

DriverStim300::DriverStim300(boost::asio::io_context &io_context,
                             const std::string &device,
                             DatagramIdentifier datagram_id,
                             GyroOutputUnit gyro_output_unit,
                             AccOutputUnit acc_output_unit,
                             InclOutputUnit incl_output_unit,
                             AccRange acc_range, SampleFreq freq)
    : serial_port_(io_context, device),
      datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit,
                       incl_output_unit, acc_range),
      datagram_id_(datagram_identifier_to_raw(datagram_id)),
      crc_dummy_bytes_(number_of_padding_bytes(datagram_id)),
      datagram_size_(calculate_datagram_size(datagram_id)),
      sensor_config_{'0',
                     0,
                     freq,
                     datagram_id,
                     false,
                     gyro_output_unit,
                     acc_output_unit,
                     incl_output_unit,
                     acc_range} {
  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(921600));
  serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_port_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
  serial_port_.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
}

DriverStim300::DriverStim300(boost::asio::io_context &io_context,
                             const std::string &device)
    : DriverStim300(io_context, device,
                    DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
                    GyroOutputUnit::AVERAGE_ANGULAR_RATE,
                    AccOutputUnit::AVERAGE_ACCELERATION,
                    InclOutputUnit::AVERAGE_ACCELERATION, AccRange::G5,
                    SampleFreq::S125) {}

DriverStim300::~DriverStim300() { stop(); }

void DriverStim300::start_async_read(StatusHandler status_handler) {
  status_handler_ = std::move(status_handler);
  ::tcflush(serial_port_.native_handle(), TCIOFLUSH);
  ask_for_config_datagram();
  async_read();
}

void DriverStim300::stop() {
  boost::system::error_code ignored;
  serial_port_.cancel(ignored);
  serial_port_.close(ignored);
}

void DriverStim300::async_read() {
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code &error, std::size_t bytes_read) {
        read_data_stream(error, bytes_read);
      });
}

void DriverStim300::read_data_stream(const boost::system::error_code &error,
                                     std::size_t bytes_read) {
  if (error == boost::asio::error::operation_aborted)
    return;
  if (error) {
    if (status_handler_)
      status_handler_(Stim300Status::ERROR);
    return;
  }

  pending_bytes_.insert(pending_bytes_.end(), read_buffer_.begin(),
                        read_buffer_.begin() + bytes_read);
  process_pending_data();
  async_read();
}

void DriverStim300::process_pending_data() {
  while (!pending_bytes_.empty()) {
    const auto id = pending_bytes_.front();
    const auto config_id =
        datagram_identifier_to_raw(DatagramIdentifier::CONFIGURATION);
    const auto config_crlf_id =
        datagram_identifier_to_raw(DatagramIdentifier::CONFIGURATION_CRLF);

    if (id != datagram_id_ && id != config_id && id != config_crlf_id) {
      pending_bytes_.erase(pending_bytes_.begin());
      continue;
    }

    const auto format = raw_to_datagram_identifier(id);
    const auto datagram_size = calculate_datagram_size(format);
    const bool has_crlf =
        id == config_crlf_id ||
        (id == datagram_id_ && sensor_config_.normal_datagram_crlf);
    const std::size_t total_size = datagram_size + (has_crlf ? 2 : 0);
    if (pending_bytes_.size() < total_size)
      return;

    if (has_crlf && (pending_bytes_[datagram_size] != '\r' ||
                     pending_bytes_[datagram_size + 1] != '\n')) {
      pending_bytes_.erase(pending_bytes_.begin());
      continue;
    }

    const auto padding = number_of_padding_bytes(format);
    if (!verify_checksum(pending_bytes_.data(), datagram_size, padding)) {
      pending_bytes_.erase(pending_bytes_.begin());
      continue;
    }

    const auto saved_id = datagram_id_;
    const auto saved_size = datagram_size_;
    const auto saved_padding = crc_dummy_bytes_;
    datagram_id_ = id;
    datagram_size_ = datagram_size;
    crc_dummy_bytes_ = number_of_padding_bytes(format);
    const auto status = parse_datagram();
    if (id == config_id || id == config_crlf_id) {
      // parse_datagram installs the format reported by the configuration.
    } else {
      datagram_id_ = saved_id;
      datagram_size_ = saved_size;
      crc_dummy_bytes_ = saved_padding;
    }

    pending_bytes_.erase(pending_bytes_.begin(),
                         pending_bytes_.begin() + total_size);
    if (status_handler_ && status != Stim300Status::NORMAL)
      status_handler_(status);
  }
}

Stim300Status DriverStim300::parse_datagram() {
  const auto config_id =
      datagram_identifier_to_raw(DatagramIdentifier::CONFIGURATION);
  const auto config_crlf_id =
      datagram_identifier_to_raw(DatagramIdentifier::CONFIGURATION_CRLF);
  if (datagram_id_ == config_id || datagram_id_ == config_crlf_id) {
    const auto config = datagram_parser_.parse_config(pending_bytes_.data());
    const auto status = config != sensor_config_ ? Stim300Status::CONFIG_CHANGED
                                                 : Stim300Status::NORMAL;
    sensor_config_ = config;
    set_datagram_format(config.datagram_id);
    datagram_parser_.set_data_parameters(config);
    return status;
  }

  sensor_data_ = datagram_parser_.parse_data(pending_bytes_.data());
  sensor_status_ = sensor_data_.status;
  if (sensor_status_ == 0)
    return Stim300Status::NEW_MEASUREMENT;
  if (sensor_status_ & (1u << 6u))
    return Stim300Status::STARTING_SENSOR;
  if (sensor_status_ & (1u << 7u))
    return Stim300Status::SYSTEM_INTEGRITY_ERROR;
  if (sensor_status_ & (1u << 5u))
    return Stim300Status::OUTSIDE_OPERATING_CONDITIONS;
  if (sensor_status_ & (1u << 4u))
    return Stim300Status::OVERLOAD;
  if (sensor_status_ & (1u << 3u))
    return Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL;
  return Stim300Status::ERROR;
}

void DriverStim300::ask_for_config_datagram() {
  static constexpr std::array<uint8_t, 2> command{'C', '\r'};
  boost::asio::async_write(
      serial_port_, boost::asio::buffer(command),
      [this](const boost::system::error_code &error, std::size_t) {
        if (error && status_handler_)
          status_handler_(Stim300Status::ERROR);
      });
}

void DriverStim300::set_datagram_format(DatagramIdentifier id) {
  datagram_id_ = datagram_identifier_to_raw(id);
  datagram_size_ = calculate_datagram_size(id);
  crc_dummy_bytes_ = number_of_padding_bytes(id);
}

bool DriverStim300::verify_checksum(const uint8_t *begin, std::size_t size,
                                    uint8_t crc_dummy_bytes) {
  const auto crc = stim_300::DatagramParser::parse_crc(begin + size - 4);
  std::vector<uint8_t> crc_data(begin, begin + size - 4);
  crc_data.resize(crc_data.size() + crc_dummy_bytes, 0);
  boost::crc_basic<32> calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
  calculator.process_bytes(crc_data.data(), crc_data.size());
  return calculator.checksum() == crc;
}

double DriverStim300::get_acc_x() const noexcept { return sensor_data_.acc[0]; }
double DriverStim300::get_acc_y() const noexcept { return sensor_data_.acc[1]; }
double DriverStim300::get_acc_z() const noexcept { return sensor_data_.acc[2]; }
double DriverStim300::get_gyro_x() const noexcept {
  return sensor_data_.gyro[0];
}
double DriverStim300::get_gyro_y() const noexcept {
  return sensor_data_.gyro[1];
}
double DriverStim300::get_gyro_z() const noexcept {
  return sensor_data_.gyro[2];
}
double DriverStim300::get_inc_x() const noexcept {
  return sensor_data_.incl[0];
}
double DriverStim300::get_inc_y() const noexcept {
  return sensor_data_.incl[1];
}
double DriverStim300::get_inc_z() const noexcept {
  return sensor_data_.incl[2];
}
uint16_t DriverStim300::get_sample_rate() const noexcept {
  return sample_freq_to_int(sensor_config_.sample_freq);
}
uint16_t DriverStim300::get_latency_us() const noexcept {
  return sensor_data_.latency_us;
}
bool DriverStim300::is_sensor_status_good() const noexcept {
  return sensor_status_ == 0;
}
uint8_t DriverStim300::get_internal_measurement_counter() const noexcept {
  return sensor_data_.counter;
}
double DriverStim300::get_average_temp() const noexcept {
  return std::numeric_limits<double>::quiet_NaN();
}

std::string DriverStim300::print_sensor_config() const noexcept {
  std::stringstream stream;
  stream << "Firmware: " << sensor_config_.revision
         << std::to_string(sensor_config_.firmware_version)
         << ", sample rate: " << get_sample_rate() << " Hz";
  return stream.str();
}
