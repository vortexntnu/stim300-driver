#ifndef DRIVER_STIM300_SERIAL_UBUNTU_H
#define DRIVER_STIM300_SERIAL_UBUNTU_H

#include <string>
#include "serial_driver.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <string.h>
#include <stdexcept>

class SerialUnix : public SerialDriver
{
public:
  explicit SerialUnix(const std::string& serial_port_name);

  ~SerialUnix() override;

  void open(BAUDRATE baudrate) override;

  void close() override;

  void writeByte(uint8_t byte) override;

  bool readByte(uint8_t& byte) override;

private:
  int file_handle_;
  struct termios config_;
};

#endif  // DRIVER_STIM300_SERIAL_UBUNTU_H
