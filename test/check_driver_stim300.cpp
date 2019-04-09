
#include "mock_serial_driver.h"
#include "gmock/gmock.h"
#include "../include/driver_stim300.h"

using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::SetArgReferee;

TEST(DriverTest, Test1)
{
  MockStim300SerialDriver serial_driver;
  DatagramBuffer datagram_buffer;

  ::testing::InSequence dummy;  // Expect_calls must happen in specified sequence

  EXPECT_CALL(serial_driver, open(_)).Times(1);

  EXPECT_CALL(serial_driver, readByte(_))
      .Times(63)
      .WillRepeatedly(Invoke(&datagram_buffer, &DatagramBuffer::getNextByte));

  EXPECT_CALL(serial_driver, close()).Times(1);

  DriverStim300 driverStim300(serial_driver, stim_300::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX);
  EXPECT_TRUE(driverStim300.processPacket());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroX());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroY());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroZ());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccX());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccY());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccZ());

  EXPECT_EQ(0, driverStim300.getLatency_us());
  EXPECT_EQ(0, driverStim300.getInternalMeasurmentCounter());
}

int main(int argc, char** argv)
{
  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}