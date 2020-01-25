
#include "serial_unix.h"

// Everything is learned from
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// "termios is the newer (now already a few decades old) Unix API for terminal I/O"

SerialUnix::SerialUnix(const std::string& serial_port_name)
{
  file_handle_ = ::open(serial_port_name.data(), O_RDWR | O_NOCTTY | O_NDELAY);
}

SerialUnix::~SerialUnix()
{
  close();
}

void SerialUnix::open(BAUDRATE baudrate)
{
  if (file_handle_ == -1)
  {
    throw std::runtime_error{ "Failed to open port" };
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
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config_.c_cc[VMIN] = 1;
  config_.c_cc[VTIME] = 0;
  //
  // Communication speed (simple version, using the predefined
  // constants)
  //
  bool failure;
  switch (baudrate)
  {
    case BAUDRATE::BAUD_115200:  // 115200:
      failure = (cfsetispeed(&config_, B115200) < 0 || cfsetospeed(&config_, B115200) < 0);
      break;
    case BAUDRATE::BAUD_57600:  // 57600:
      failure = (cfsetispeed(&config_, B57600) < 0 || cfsetospeed(&config_, B57600) < 0);
      break;
    case BAUDRATE::BAUD_921600:  // 921600:
      failure = (cfsetispeed(&config_, B921600) < 0 || cfsetospeed(&config_, B921600) < 0);
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
  if (tcsetattr(file_handle_, TCSAFLUSH, &config_) < 0)
  {
    throw std::runtime_error{ "Could not apply serial settings" };
  }
}

void SerialUnix::close()
{
  ::close(file_handle_);
}

void SerialUnix::writeByte(uint8_t byte)
{
  ::write(file_handle_, &byte, 1);
}

bool SerialUnix::readByte(uint8_t& byte)
{
  return (read(file_handle_, &byte, 1) > 0);
}

bool SerialUnix::flush()
{
  return tcflush(file_handle_, TCIOFLUSH) == 0;
}