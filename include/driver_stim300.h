
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include <boost/crc.hpp>
#include "../src/stim300_constants.h"
#include "../src/datagram_parser.h"
#include "../src/serial_driver.h"
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
    explicit DriverStim300(SerialDriver& serial_driver,
                  DatagramIdentifier datagram_id = DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
                  GyroOutputUnit gyro_output_unit = GyroOutputUnit::ANGULAR_RATE,
                  AccOutputUnit acc_output_unit = AccOutputUnit::ACCELERATION,
                  InclOutputUnit incl_output_unit = InclOutputUnit::ACCELERATION,
                  AccRange acc_range = AccRange::G5,
                  SampleFreq freq = SampleFreq::S250,
                  bool read_config_from_sensor = true) noexcept;
    ~DriverStim300() = default;
    //The class is Non-Copyable
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
    Mode mode_;
    enum class ReadingMode
    {
      IdentifyingDatagram,
      ReadingDatagram,
      VerifyingDatagramCR,
      VerifyingDatagramLF
    };
    ReadingMode reading_mode_;
    bool read_config_from_sensor_;

    SerialDriver& serial_driver_;
    std::vector<uint8_t> buffer_;
    size_t n_new_bytes_;
    size_t n_checked_bytes;

    uint8_t datagram_id_;
    stim_300::DatagramParser datagram_parser_;
    uint8_t datagram_size_;
    uint8_t crc_dummy_bytes_;

    stim_300::SensorConfig sensor_config_;
    stim_300::SensorData sensor_data_;
    uint8_t sensor_status_;

    Stim300Status readDataStream();
    bool setDatagramFormat(DatagramIdentifier id);
    static bool verifyChecksum(
        const std::vector<uint8_t>::const_iterator& begin,
        const std::vector<uint8_t>::const_iterator& end,
        const uint8_t& crc_dummy_bytes);

    void askForConfigDatagram();
  };

#endif  // DRIVER_STIM300_DRIVER_STIM300_H
