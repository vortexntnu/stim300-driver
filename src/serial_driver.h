
#ifndef DRIVER_STIM300_SERIAL_DRIVER_H
#define DRIVER_STIM300_SERIAL_DRIVER_H

class SerialDriver
{
public:
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
  virtual void open(BAUDRATE baudrate) = 0;
  virtual void close() = 0;
  virtual bool readByte(uint8_t& byte) = 0;
  virtual void writeByte(uint8_t byte) = 0;
  virtual ~SerialDriver() = default;
};

#endif  // DRIVER_STIM300_SERIAL_DRIVER_H
