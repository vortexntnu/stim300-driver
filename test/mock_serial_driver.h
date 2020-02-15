
#ifndef DRIVER_STIM300_MOCK_SERIAL_DRIVER_H
#define DRIVER_STIM300_MOCK_SERIAL_DRIVER_H

#include <boost/crc.hpp>
#include "gmock/gmock.h"
#include "../src/serial_unix.h"
#include "../src/stim300_constants.h"

class MockStim300SerialDriver : public SerialDriver
{
public:
  MOCK_METHOD1(readByte, bool(uint8_t& byte));
  MOCK_METHOD1(writeByte, bool(uint8_t byte));
  MOCK_METHOD0(flush, bool());

private:
  MOCK_METHOD0(close, void());
};

class DatagramBuffer
{
public:
  DatagramBuffer()
  {
    // Create a simple datagram with crc checksum
    datagram_.push_back(175);
    for (int i = 1; i < 63; ++i)
    {
      datagram_.push_back(0);
    }
    auto end = datagram_.cend();
    auto begin = datagram_.cbegin();
    uint8_t crc_dummy_bytes = stim_const::numberOfPaddingBytes(stim_const::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX);
    boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
    uint8_t buffer_CRC[end - begin - sizeof(uint32_t) + crc_dummy_bytes];
    std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes, buffer_CRC);

    /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
    for (size_t i = 0; i < crc_dummy_bytes; ++i)
      buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

    crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));
    uint32_t crc = crc_32_calculator.checksum();

    datagram_[59] = (crc & 0xff000000u) >> 24u;
    datagram_[60] = (crc & 0x00ff0000u) >> 16u;
    datagram_[61] = (crc & 0x0000ff00u) >> 8u;
    datagram_[62] = (crc & 0x000000ffu);

    it_ = datagram_.cbegin();
  }
  bool getNextByte(uint8_t& byte)
  {
    byte = *it_++;
    return true;
  }

private:
  std::vector<uint8_t>::const_iterator it_;
  std::vector<uint8_t> datagram_;
};

#endif  // DRIVER_STIM300_MOCK_SERIAL_DRIVER_H
