
#ifndef DRIVER_STIM300_SERIAL_DRIVER_H
#define DRIVER_STIM300_SERIAL_DRIVER_H

class SerialDriver
{
public:
  virtual bool readByte(uint8_t& byte) = 0;
  virtual void writeByte(uint8_t byte) = 0;
  virtual bool flush() = 0;
};

#endif  // DRIVER_STIM300_SERIAL_DRIVER_H
