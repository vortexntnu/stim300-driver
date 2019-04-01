
#include "serial_unix.h"

// Everything is learned from
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// "termios is the newer (now already a few decades old) Unix API for terminal I/O"

SerialUnix::SerialUnix(const std::string& serial_port_name)
{
  file_handle_ = ::open(serial_port_name.data(), O_RDWR | O_NOCTTY | O_NDELAY);
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
}

SerialUnix::~SerialUnix()
{
  close();
}

void SerialUnix::open(BAUDRATE baudrate)
{
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
    case BAUDRATE::BAUD_57600:  // 460800:
      failure = (cfsetispeed(&config_, B460800) < 0 || cfsetospeed(&config_, B460800) < 0);
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
    throw std::runtime_error{ "Could not apply settings" };
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

bool SerialUnix::readByte(uint8_t& byte, unsigned int ms_timeout)
{
  return (read(file_handle_, &byte, 1) > 0);
}

// void SerialUnix::open(BAUDRATE baudrate)
//{
//  struct termios options;
//
//  FD_WES_ = ::open(serial_port_name_.data(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
//  if (FD_WES_ == -1)
//  {
//    return;
//  }
//
//  bzero(&options, sizeof(options));  //?
//
//  options.c_cflag |= (CLOCAL | CREAD);
//  options.c_cflag &= ~CSIZE;  // the bitmask
//
//  // Set the baudrate
//  switch (baudrate)
//  {
//    case BAUDRATE::BAUD_115200:  // 115200:
//      cfsetispeed(&options, B115200);
//      cfsetospeed(&options, B115200);
//      break;
//    case BAUDRATE::BAUD_57600:  // 460800:
//      cfsetispeed(&options, B460800);
//      cfsetospeed(&options, B460800);
//      break;
//    case BAUDRATE::BAUD_921600:  // 921600:
//      cfsetispeed(&options, B921600);
//      cfsetospeed(&options, B921600);
//      break;
//    default:
//      cfsetispeed(&options, B460800);
//      cfsetospeed(&options, B460800);
//  }
//
//  options.c_cflag |= CS8;  // 8 data bits
//
//  options.c_cflag &= ~PARENB;
//  options.c_iflag &= ~INPCK;  // no parity check
//
//  options.c_cflag &= ~CSTOPB;  // one stop bit
//
//  options.c_cc[VTIME] = 0;  // waiting time for reading every bit
//  options.c_cc[VMIN] = 0;   // minimum bit(s) to readChunk
//
//  options.c_lflag &= ~(ICANON | ECHO | ISIG);
//
//  tcflush(0, TCIOFLUSH);
//
//  // activate the configuration
//  if ((tcsetattr(FD_WES_, TCSANOW, &options)) != 0)
//  {
//    cout << "com setup error!" << endl;
//    return;
//  }
//  else
//  {
//    cout << "com setup succeeded!" << endl;
//    return;
//  }
//}
//
// bool SerialUnix::readByte(uint8_t& byte, unsigned int ms_timeout)
//{
//  uint8_t* buff;
//  if (read(FD_WES_, buff, 1) and buff != nullptr)
//  {
//    byte = *buff;
//    return true;
//  }
//  return false;
//}
//
// void SerialUnix::close()
//{
//}
// SerialUnix::~SerialUnix()
//{
//  close();
//}
//
// void SerialUnix::write(const std::vector<unsigned char>& data_buffer)
//{
//}
