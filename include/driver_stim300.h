
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include <boost/crc.hpp>
#include "../src/stim300_constants.h"
#include "../src/datagram_parser.h"
#include "../src/serial_unix.h"
#include <vector>

enum class Stim300Status
{
  NORMAL,
  NEW_MEASURMENT,
  CONFIG_CHANGED,
  STARTING_SENSOR,
  SYSTEM_INTEGRITY_ERROR,
  OUTSIDE_OPERATING_CONDITIONS,
  OVERLOAD,
  ERROR_IN_MEASUREMENT_CHANNEL,
  ERROR
};

class DriverStim300
{
public:
  DriverStim300(SerialDriver& serial_driver, DatagramIdentifier datagram_id, GyroOutputUnit gyro_output_unit,
                AccOutputUnit acc_output_unit, InclOutputUnit incl_output_unit, AccRange acc_range, SampleFreq freq);
  explicit DriverStim300(SerialDriver& serial_driver);
  ~DriverStim300() = default;
  // The class is Non-Copyable
  DriverStim300(const DriverStim300& a) = delete;
  DriverStim300& operator=(const DriverStim300& a) = delete;
  // The class is non-movable
  DriverStim300(DriverStim300&& a) = delete;
  DriverStim300& operator=(DriverStim300&& a) = delete;

  double getAccX() const noexcept;
  double getAccY() const noexcept;
  double getAccZ() const noexcept;
  double getGyroX() const noexcept;
  double getGyroY() const noexcept;
  double getGyroZ() const noexcept;
  uint16_t getLatency_us() const noexcept;
  double getAverageTemp() const noexcept;
  std::string printSensorConfig() const noexcept;
  bool isSensorStatusGood() const noexcept;
  uint8_t getInternalMeasurementCounter() const noexcept;
  Stim300Status update() noexcept;

private:
  enum class Mode : uint8_t
  {
    Init,
    Normal,
    Service
  };
  Mode mode_{ Mode::Init };
  enum class ReadingMode
  {
    IdentifyingDatagram,
    ReadingDatagram,
    VerifyingDatagramCR,
    VerifyingDatagramLF
  };
  ReadingMode reading_mode_{ ReadingMode::IdentifyingDatagram };

  SerialDriver& serial_driver_;
  stim_300::DatagramParser datagram_parser_;
  std::vector<uint8_t> buffer_{};
  size_t n_new_bytes_{ 0 };
  size_t n_checked_bytes{ 0 };

  uint8_t datagram_id_;
  uint8_t crc_dummy_bytes_;
  uint8_t datagram_size_;
  stim_300::SensorConfig sensor_config_;
  bool read_config_from_sensor_{ true };
  stim_300::SensorData sensor_data_{};
  uint8_t sensor_status_{ 0 };

  Stim300Status readDataStream();
  bool setDatagramFormat(DatagramIdentifier id);
  static bool verifyChecksum(const std::vector<uint8_t>::const_iterator& begin,
                             const std::vector<uint8_t>::const_iterator& end, const uint8_t& crc_dummy_bytes);

  void askForConfigDatagram();
};

#endif  // DRIVER_STIM300_DRIVER_STIM300_H
