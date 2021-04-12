
#include "serial_unix.h"
#include <iostream>
// Everything is learned from
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// "termios is the newer (now already a few decades old) Unix API for terminal I/O"

SerialUnix::SerialUnix(const std::string& serial_port_name, stim_const::BaudRate baudrate)
{
  open(serial_port_name, baudrate);
}

SerialUnix::~SerialUnix()
{
  close();
}

void SerialUnix::open(const std::string& serial_port_name, stim_const::BaudRate baudrate)
{
  file_handle_ = ::open(serial_port_name.data(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (file_handle_ < 0)
  {
    throw std::runtime_error{ std::string{ strerror(errno) } + ": " + serial_port_name };
  }
  //
  // Check if the file descriptor is pointing to a TTY device or not.
  //
  if (!isatty(file_handle_))
  {
    throw std::runtime_error{ "Serial port is not TTY device" };
  }

  //
  // Get the current configuration of the serial interface
  //
  if (tcgetattr(file_handle_, &config_) < 0)
  {
    throw std::runtime_error{ "Could not retrive current serial config" };
  }

  //
  // Input flags - Turn off input processing
  //
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config_.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  //
  // Output flags - Turn off output processing
  //
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  // config_.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
  //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
  config_.c_oflag = 0;

  //
  // No line processing
  //
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config_.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  //
  // Turn off character processing
  //
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config_.c_cflag &= ~(CSIZE | PARENB);
  config_.c_cflag |= CS8;

  //
  // Zero input byte is enough to return from read()
  // Inter-character timer off
  // I.e. no blocking: return immediately with what is available.
  //
  config_.c_cc[VMIN] = 0;
  config_.c_cc[VTIME] = 0;
  //
  // Communication speed (simple version, using the predefined
  // constants)
  //
  bool failure;
  switch (baudrate)
  {
    case stim_const::BaudRate::BAUD_377400:
      failure = (cfsetispeed(&config_, 377400) < 0 || cfsetospeed(&config_, 377400) < 0);
      break;
    case stim_const::BaudRate::BAUD_460800:
      failure = (cfsetispeed(&config_, B460800) < 0 || cfsetospeed(&config_, B460800) < 0);
      break;
    case stim_const::BaudRate::BAUD_921600:
      failure = (cfsetispeed(&config_, B921600) < 0 || cfsetospeed(&config_, B921600) < 0);
      break;
    case stim_const::BaudRate::BAUD_1843200:  // 921600:
      failure = (cfsetispeed(&config_, 1843200) < 0 || cfsetospeed(&config_, 1843200) < 0);
      break;
    default:
      failure = true;
  }
  if (failure)
  {
    throw std::runtime_error{ "Could not set baudrate" };
  }

  //
  // Finally, apply the configuration
  //
  if (tcsetattr(file_handle_, TCSAFLUSH, &config_) != 0)
  {
    throw std::runtime_error{ strerror(errno) };
  }
}

void SerialUnix::close()
{
  ::close(file_handle_);
}

bool SerialUnix::writeByte(uint8_t byte)
{
  int result = ::write(file_handle_, &byte, 1);
  if (result == -1)
  {
    throw std::runtime_error{ "WriteByte error:" + std::string{ strerror(errno) } };
  }
  return result == 1;
}

bool SerialUnix::readByte(uint8_t& byte)
{
  int result = read(file_handle_, &byte, 1);
  if (result == -1)
  {
    throw std::runtime_error{ "ReadByte error:" + std::string{ strerror(errno) } };
  }
  return (result == 1);
}

bool SerialUnix::flush()
{
  return tcflush(file_handle_, TCIOFLUSH) == 0;
}