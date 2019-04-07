#include <iostream>
#include "driver_stim300.h"
#include "serial_unix.h"

int main()
{
  SerialUnix serial_driver("/dev/ttyUSB0");
  DriverStim300 driver_stim300(serial_driver);
  uint8_t last_count{ 0 };
  for (int i = 0; i < 5000; ++i)
  {
    usleep(4000);
    if (!driver_stim300.processPacket())
    {
      // std::cout << "Could not process packet" << std::endl;
      continue;
    }

    if (!driver_stim300.isChecksumGood())
    {
      std::cout << "Checksum_error" << std::endl;
      continue;
    }

    std::cout << "ok" << std::endl;
    std::cout << "Acc: " << std::fixed << driver_stim300.getAccX() << "," << driver_stim300.getAccY() << ","
              << driver_stim300.getAccZ() << std::endl;
    std::cout << "Gyro: " << std::fixed << driver_stim300.getGyroX() << "," << driver_stim300.getGyroY() << ","
              << driver_stim300.getGyroZ() << std::endl;
    std::cout << "Good?: " << std::fixed << driver_stim300.isSensorStatusGood() << std::endl;
    std::cout << "Latency: " << driver_stim300.getLatency_us() << std::endl;
    std::cout << "Counter diff: " << (uint)(driver_stim300.getInternalMeasurmentCounter() - last_count) << std::endl;
    last_count = driver_stim300.getInternalMeasurmentCounter();
  }
  return 0;
}