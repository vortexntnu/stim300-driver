#ifndef DRIVER_STIM300_SERIAL_UBUNTU_H
#define DRIVER_STIM300_SERIAL_UBUNTU_H

#include <string>
#include "serial_driver.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include "stim300_constants.h"

class SerialUnix : public SerialDriver
{
public:
  SerialUnix(const std::string& serial_port_name, stim_const::BaudRate baudrate);
  ~SerialUnix() final;
  // The class is Non-Copyable
  SerialUnix(const SerialUnix& a) = delete;
  SerialUnix& operator=(const SerialUnix& a) = delete;
  // The class is non-movable
  SerialUnix(SerialUnix&& a) = delete;
  SerialUnix& operator=(SerialUnix&& a) = delete;

  bool writeByte(uint8_t byte) final;
  bool readByte(uint8_t& byte) final;
  bool flush() final;

private:
  void open(const std::string& serial_port_name, stim_const::BaudRate baudrate);
  void close();

  int file_handle_;
  struct termios config_;
};

#endif  // DRIVER_STIM300_SERIAL_UBUNTU_H
