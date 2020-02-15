
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
  EXPECT_CALL(serial_driver, flush()).Times(1);
  EXPECT_CALL(serial_driver, writeByte('C')).Times(1);
  EXPECT_CALL(serial_driver, writeByte('\r')).Times(1);

  EXPECT_CALL(serial_driver, readByte(_))
    .Times(63)
    .WillRepeatedly(Invoke(&datagram_buffer, &DatagramBuffer::getNextByte));

  // Only created a mock datagram for RATE_ACC_INCL_TEMP_AUX with all values equal to zero
  // TODO: Create more advanced mock datagrams
  DriverStim300 driverStim300(serial_driver, stim_const::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
                              GyroOutputUnit::ANGULAR_RATE, AccOutputUnit::ACCELERATION, InclOutputUnit::ACCELERATION,
                              AccRange::G5, SampleFreq::S125);

  EXPECT_EQ(Stim300Status::NEW_MEASURMENT, driverStim300.update());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroX());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroY());
  EXPECT_DOUBLE_EQ(0, driverStim300.getGyroZ());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccX());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccY());
  EXPECT_DOUBLE_EQ(0, driverStim300.getAccZ());

  EXPECT_EQ(0, driverStim300.getLatency_us());
  EXPECT_EQ(0, driverStim300.getInternalMeasurementCounter());
}

int main(int argc, char** argv)
{
  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}