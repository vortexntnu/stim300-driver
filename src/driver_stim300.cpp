
#include <iostream>
#include "driver_stim300.h"

DriverStim300::DriverStim300(SerialDriver& serial_driver, DatagramIdentifier datagram_id,
                             GyroOutputUnit gyro_output_unit, AccOutputUnit acc_output_unit,
                             InclOutputUnit incl_output_unit, AccRange acc_range, SampleFreq freq,
                             bool read_config_from_sensor) noexcept
  : serial_driver_(serial_driver)
  , datagram_id_(datagramIdentifierToRaw(datagram_id))
  , mode_(Mode::Init)
  , reading_mode_(ReadingMode::IdentifyingDatagram)
  , n_new_bytes_(0)
  , sensor_status_(0)
  , crc_dummy_bytes_(numberOfPaddingBytes(datagram_id))
  , sensor_config_{ '0', 0, freq, datagram_id, false, gyro_output_unit, acc_output_unit, incl_output_unit, acc_range }
  , sensor_data_()
  , datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit, incl_output_unit, acc_range)
  , datagram_size_(calculateDatagramSize(datagram_id))
  , read_config_from_sensor_(read_config_from_sensor)
{
}

double DriverStim300::getAccX() const noexcept
{
  return sensor_data_.acc[0];
}
double DriverStim300::getAccY() const noexcept
{
  return sensor_data_.acc[1];
}
double DriverStim300::getAccZ() const noexcept
{
  return sensor_data_.acc[2];
}
double DriverStim300::getGyroX() const noexcept
{
  return sensor_data_.gyro[0];
}
double DriverStim300::getGyroY() const noexcept
{
  return sensor_data_.gyro[1];
}
double DriverStim300::getGyroZ() const noexcept
{
  return sensor_data_.gyro[2];
}
uint16_t DriverStim300::getLatency_us() const noexcept
{
  return sensor_data_.latency_us;
}
bool DriverStim300::isSensorStatusGood() const noexcept
{
  return sensor_status_ == 0;
}
uint8_t DriverStim300::getInternalMeasurementCounter() const noexcept
{
  return sensor_data_.counter;
}

double DriverStim300::getAverageTemp() const noexcept
{
  double sum{ 0 };
  uint8_t count{ 0 };

  return count != 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
}

Stim300Status DriverStim300::readDataStream()
{
  // Read stream to identify the start of a datagram.
  // If no datagram is identified after 100 bytes, ask for a config datagram.
  // Read number of bytes belonging to the current datagram.
  // Verify datagram (CRLF and CRC).
  // Parse Datagram.
  uint8_t byte;
  while (serial_driver_.readByte(byte))
  {
    switch (reading_mode_)
    {
      case ReadingMode::IdentifyingDatagram:
        if (byte == datagram_id_)
        {
          // Use current datagram format
        }
        else if (byte == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION))
        {
          setDatagramFormat(DatagramIdentifier::CONFIGURATION);
        }
        else if (byte == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF))
        {
          setDatagramFormat(DatagramIdentifier::CONFIGURATION_CRLF);
        }
        else
        {
          if (++n_checked_bytes > 100)
          {
            std::cerr << "Not able to recognise datagram" << std::endl;
            if (read_config_from_sensor_)
              askForConfigDatagram();
            n_checked_bytes = 0;
          }
          continue;
        }
        if (n_checked_bytes != 0)
          std::cout << "Checked bytes: " << n_checked_bytes << std::endl;
        n_checked_bytes = 0;
        reading_mode_ = ReadingMode::ReadingDatagram;
        buffer_.push_back(byte);
        n_new_bytes_ = 1;
        continue;

      case ReadingMode::ReadingDatagram:

        // Circular buffer
        buffer_.push_back(byte);
        n_new_bytes_++;
        while (buffer_.size() > datagram_size_)
          buffer_.erase(buffer_.begin());

        if (n_new_bytes_ < datagram_size_)
          continue;

        // else the buffer is filled with a potential new datagram

        if (sensor_config_.normal_datagram_CRLF or
            datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF))
        {
          reading_mode_ = ReadingMode::VerifyingDatagramCR;
          continue;
        }
        break;

      case ReadingMode::VerifyingDatagramCR:

        if (byte == 0x0D)
        {
          reading_mode_ = ReadingMode::VerifyingDatagramLF;
          continue;
        }
        reading_mode_ = ReadingMode::IdentifyingDatagram;
        return Stim300Status::NORMAL;

      case ReadingMode::VerifyingDatagramLF:

        if (byte == 0x0A)
        {
          break;
        }
        reading_mode_ = ReadingMode::IdentifyingDatagram;
        return Stim300Status::NORMAL;
    }  // end reading mode switch

    if (!verifyChecksum(buffer_.cbegin(), buffer_.cend(), crc_dummy_bytes_))
    {
      // The "ID" was likely a byte happening to be equal the datagram id,
      // and not actually the start of a datagram, thus the buffer does
      // not contain a complete datagram.
      std::cerr << "CRC error" << std::endl;
      reading_mode_ = ReadingMode::IdentifyingDatagram;
      return Stim300Status::NORMAL;
    }

    if (datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF) or
        datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION))
    {
      stim_300::SensorConfig sensor_config = datagram_parser_.parseConfig(buffer_.cbegin());
      Stim300Status status{ Stim300Status::NORMAL };
      if (sensor_config != sensor_config_)
        status = Stim300Status::CONFIG_CHANGED;

      sensor_config_ = sensor_config;
      setDatagramFormat(sensor_config_.datagram_id);
      datagram_parser_.setDataParameters(sensor_config_);
      read_config_from_sensor_ = false;
      std::cout << "Parsed config datagram" << std::endl;
      reading_mode_ = ReadingMode::IdentifyingDatagram;
      return status;
    }
    else  // Normal (measurement) datagram
    {
      sensor_status_ = datagram_parser_.parseData(buffer_.cbegin(), sensor_data_);
      reading_mode_ = ReadingMode::IdentifyingDatagram;

      if (sensor_status_ == 0)
        return Stim300Status::NEW_MEASURMENT;
      else if (sensor_status_ & (1u << 6u))
        return Stim300Status::STARTING_SENSOR;
      else if (sensor_status_ & (1u << 7u))
        return Stim300Status::SYSTEM_INTEGRITY_ERROR;
      else if (sensor_status_ & (1u << 5u))
        return Stim300Status::OUTSIDE_OPERATING_CONDITIONS;
      else if (sensor_status_ & (1u << 4u))
        return Stim300Status::OVERLOAD;
      else if (sensor_status_ & (1u << 3u))
        return Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL;
      else
        return Stim300Status::ERROR;
    }
  }
  return Stim300Status::NORMAL;
}

void DriverStim300::askForConfigDatagram()
{
  serial_driver_.writeByte('C');
  serial_driver_.writeByte(0x0D);
}

Stim300Status DriverStim300::update() noexcept
{
  switch (mode_)
  {
    case Mode::Init:
      serial_driver_.flush();
      if (read_config_from_sensor_)
        askForConfigDatagram();
      mode_ = Mode::Normal;
    case Mode::Normal:
      return readDataStream();
    case Mode::Service:
      // std::string s(buffer_.begin(), buffer_.end());
      // std::cout << s << "\n";
      mode_ = Mode::Normal;
      return Stim300Status::NORMAL;
  }
}

bool DriverStim300::setDatagramFormat(DatagramIdentifier id)
{
  datagram_id_ = datagramIdentifierToRaw(id);
  datagram_size_ = calculateDatagramSize(id);
  crc_dummy_bytes_ = numberOfPaddingBytes(id);
}

bool DriverStim300::verifyChecksum(const std::vector<uint8_t>::const_iterator& begin,
                                   const std::vector<uint8_t>::const_iterator& end, const uint8_t& crc_dummy_bytes)
{
  uint32_t crc = stim_300::DatagramParser::parseCRC(end - sizeof(uint32_t));

  boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
  uint8_t buffer_CRC[end - begin - sizeof(uint32_t) + crc_dummy_bytes];
  std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes, buffer_CRC);

  /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
  for (size_t i = 0; i < crc_dummy_bytes; ++i)
    buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

  crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));

  return crc_32_calculator.checksum() == crc;
}

std::string DriverStim300::printSensorConfig() const noexcept
{
  return sensor_config_.print();
}
