#ifndef DRIVER_STIM300_SERIAL_UBUNTU_H
#define DRIVER_STIM300_SERIAL_UBUNTU_H

#include <string>
#include "serial_driver.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

enum class BAUDRATE : uint32_t
{
  BAUD_4800 = 4800,
  BAUD_9600 = 9600,
  BAUD_19200 = 19200,
  BAUD_38400 = 38400,
  BAUD_57600 = 57600,
  BAUD_115200 = 115200,
  BAUD_921600 = 921600
};

class SerialUnix : public SerialDriver
{
public:
  explicit SerialUnix(const std::string& serial_port_name);

  ~SerialUnix();

  void open(BAUDRATE baudrate);

  void close();

  void writeByte(uint8_t byte) override;

  bool readByte(uint8_t& byte) override;

  bool flush() override;

private:
  int file_handle_;
  struct termios config_;
};

#endif  // DRIVER_STIM300_SERIAL_UBUNTU_H
