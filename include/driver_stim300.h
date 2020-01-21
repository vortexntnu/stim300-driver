
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include <boost/crc.hpp>
#include "../src/stim300_constants.h"
#include "../src/datagram_parser.h"
#include "../src/serial_driver.h"
#include <vector>

using namespace stim_300;

  enum class Status
  {
    NORMAL,
    NEW_MEASURMENT,
    CONFIG_CHANGED,
    ERROR
  };

  class DriverStim300
  {
  public:
    DriverStim300(SerialDriver& serial_driver,
                  DatagramIdentifier datagram_id = DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
                  GyroOutputUnit gyro_output_unit = GyroOutputUnit::ANGULAR_RATE,
                  AccOutputUnit acc_output_unit = AccOutputUnit::ACCELERATION,
                  InclOutputUnit incl_output_unit = InclOutputUnit::ACCELERATION,
                  SerialDriver::BAUDRATE baudrate = SerialDriver::BAUDRATE::BAUD_921600,
                  uint16_t serial_read_timeout_ms = 1,
                  AccRange acc_range = AccRange::G5,
                  SampleFreq freq = SampleFreq::S250,
                  bool read_config_from_sensor = true);
    ~DriverStim300();
    double getAccX() const;
    double getAccY() const;
    double getAccZ() const;
    double getGyroX() const;
    double getGyroY() const;
    double getGyroZ() const;
    uint16_t getLatency_us() const;
    double getAverageTemp() const;
    std::string printSensorConfig() const;
    bool isChecksumGood() const;
    bool isSensorStatusGood() const;
    uint8_t getInternalMeasurmentCounter() const;
    Status update();

  private:
    enum class Mode : uint8_t
    {
      Init,
      Normal,
      Service
    };
    Mode mode_;
    enum class ReadingMode : uint8_t
    {
      IdentifyingDatagram,
      ReadingDatagram,
    };
    ReadingMode reading_mode_;
    bool read_config_from_sensor_;

    SerialDriver& serial_driver_;
    uint16_t serial_read_timeout_ms_;
    std::vector<uint8_t> buffer_;
    size_t n_new_bytes_;

    uint8_t datagram_id_;
    DatagramParser datagram_parser_;
    uint8_t datagram_size_;
    uint8_t crc_dummy_bytes_;

    SensorConfig sensor_config_;
    SensorData sensor_data_;
    bool checksum_is_ok_;
    bool no_internal_error_;

    Status readDataStream();
    bool setDatagramFormat(DatagramIdentifier id);
    bool verifyChecksum(
        std::vector<uint8_t>::const_iterator begin,
        std::vector<uint8_t>::const_iterator end);

    void askForConfigDatagram();
  };

#endif  // DRIVER_STIM300_DRIVER_STIM300_H
