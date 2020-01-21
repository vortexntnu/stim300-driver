
#include <iostream>
#include "driver_stim300.h"


DriverStim300::DriverStim300(SerialDriver& serial_driver, DatagramIdentifier datagram_id,
                             GyroOutputUnit gyro_output_unit, AccOutputUnit acc_output_unit,
                             InclOutputUnit incl_output_unit, SerialDriver::BAUDRATE baudrate,
                             uint16_t serial_read_timeout_ms, AccRange acc_range,
                             SampleFreq freq, bool read_config_from_sensor)
  : serial_driver_(serial_driver)
  , serial_read_timeout_ms_(serial_read_timeout_ms)
  , datagram_id_(datagramIdentifierToRaw(datagram_id))
  , mode_(Mode::Init)
  , reading_mode_(ReadingMode::IdentifyingDatagram)
  , n_new_bytes_(0)
  , checksum_is_ok_(false)
  , no_internal_error_(true)
  , crc_dummy_bytes_(numberOfPaddingBytes(datagram_id))
  , sensor_config_{'0',0, freq, datagram_id, gyro_output_unit, acc_output_unit, incl_output_unit, acc_range}
  , sensor_data_()
  , datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit, incl_output_unit, acc_range)
  , datagram_size_(calculateDatagramSize(datagram_id))
  , read_config_from_sensor_(read_config_from_sensor)
{
  serial_driver_.open(baudrate);
}

DriverStim300::~DriverStim300()
{
  serial_driver_.close();
}

double DriverStim300::getAccX() const
{
  return sensor_data_.acc[0];
}
double DriverStim300::getAccY() const
{
  return sensor_data_.acc[1];
}
double DriverStim300::getAccZ() const
{
  return sensor_data_.acc[2];
}
double DriverStim300::getGyroX() const
{
  return sensor_data_.gyro[0];
}
double DriverStim300::getGyroY() const
{
  return sensor_data_.gyro[1];
}
double DriverStim300::getGyroZ() const
{
  return sensor_data_.gyro[2];
}
uint16_t DriverStim300::getLatency_us() const
{
  return sensor_data_.latency_us;
}
bool DriverStim300::isChecksumGood() const
{
  return checksum_is_ok_;
}
bool DriverStim300::isSensorStatusGood() const
{
  return no_internal_error_;
}
uint8_t DriverStim300::getInternalMeasurmentCounter() const
{
  return sensor_data_.counter;
}

double DriverStim300::getAverageTemp() const
{
  double sum{ 0 };
  uint8_t count{ 0 };

  return count != 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
}

Stim300Status DriverStim300::readDataStream()
{
  // Read stream until a datagram is identified.
  // Read number of bytes belonging to the current datagram.
  // Verify datagram (CRC).
  // Parse Datagram.
  Stim300Status status {Stim300Status::NORMAL};
  uint8_t byte;
  while (serial_driver_.readByte(byte))
  {
    switch (reading_mode_)
    {
      case ReadingMode::IdentifyingDatagram :
        if (byte == datagram_id_)
        {
          // Keep current datagram id
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
          break;
        }
        reading_mode_ = ReadingMode::ReadingDatagram;
        n_new_bytes_ = 0;

      case ReadingMode::ReadingDatagram :

        // Circular buffer
        buffer_.push_back(byte);
        n_new_bytes_++;
        while (buffer_.size() > datagram_size_)
          buffer_.erase(buffer_.begin());

        if(n_new_bytes_ >= datagram_size_) // Buffer contains a new datagram
        {
          auto begin = buffer_.cbegin();

          if (!verifyChecksum(begin, begin+datagram_size_))
          {
            // The "ID" was likely a byte happening to be equal the datagram id,
            // and not actually the start of a datagram, thus the buffer does
            // not contain a complete datagram.
            reading_mode_ = ReadingMode::IdentifyingDatagram;
            return Stim300Status::NORMAL;
          }

          if (datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF) or
              datagram_id_ == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION))
          {
            stim_300::SensorConfig sensor_config{};
            datagram_parser_.parseConfig(begin, sensor_config);
            if (sensor_config != sensor_config_)
            {
              status = Stim300Status::CONFIG_CHANGED;
              sensor_config_ = sensor_config;
            }
            setDatagramFormat(sensor_config_.datagram_id);
            datagram_parser_.setDataParameters(sensor_config_);
          }
          else
          {
            no_internal_error_ = datagram_parser_.parseData(begin, sensor_data_);
            status = Stim300Status::NEW_MEASURMENT;
          }
          reading_mode_ = ReadingMode::IdentifyingDatagram;
          return status;
        }
        break;
    }
  }
}
void DriverStim300::askForConfigDatagram()
{
  serial_driver_.writeByte('C');
  serial_driver_.writeByte(0x0D);
}

Stim300Status DriverStim300::update()
{
  switch (mode_)
  {
    case Mode::Init:
      // Read Data config
      mode_ = Mode::Normal;
    case Mode::Normal:
      // Read from buffer until we find a datagram identifyer.
      // Read the amount of bytes one datagram should contain.
      // Parse that datagram.
      if (read_config_from_sensor_)
      {
        askForConfigDatagram();
        read_config_from_sensor_ = false;
      }
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


bool DriverStim300::verifyChecksum(std::vector<uint8_t>::const_iterator begin, std::vector<uint8_t>::const_iterator end)
{
  uint32_t crc = stim_300::DatagramParser::parseCRC(end - sizeof(uint32_t));

  boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
  uint8_t buffer_CRC[datagram_size_ - sizeof(uint32_t) + crc_dummy_bytes_];
  std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes_, buffer_CRC);

  /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
  for (size_t i = 0; i < crc_dummy_bytes_; ++i)
    buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

  crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));

  return crc_32_calculator.checksum() == crc;
}

std::string DriverStim300::printSensorConfig() const
{
  return sensor_config_.print();
}
