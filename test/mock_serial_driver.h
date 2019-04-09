
#ifndef DRIVER_STIM300_MOCK_SERIAL_DRIVER_H
#define DRIVER_STIM300_MOCK_SERIAL_DRIVER_H

#include "gmock/gmock.h"
#include "../src/serial_driver.h"

class MockStim300SerialDriver : public SerialDriver
{
public:
  MOCK_METHOD1(open, void(BAUDRATE baudrate));
  MOCK_METHOD0(close, void());
  MOCK_METHOD1(readByte, bool(uint8_t& byte));
  MOCK_METHOD1(writeByte,void(uint8_t byte));
};

class DatagramBuffer
{
public:
  DatagramBuffer()
  {
    datagram_.push_back(175);
    for (int i = 0; i < 62; ++i)
    {
      datagram_.push_back(0);
    }
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
