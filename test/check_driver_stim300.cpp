#include "mock_serial_driver.h"
#include "stim300_driver/datagram_parser.hpp"
#include "gtest/gtest.h"

TEST(DatagramParserTest, parses_measurement_datagram) {
  DatagramBuffer datagram;
  stim_300::DatagramParser parser(
      stim_const::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
      stim_const::GyroOutputUnit::ANGULAR_RATE,
      stim_const::AccOutputUnit::ACCELERATION,
      stim_const::InclOutputUnit::ACCELERATION, stim_const::AccRange::G5);

  const auto data = parser.parse_data(datagram.data());
  EXPECT_DOUBLE_EQ(0, data.gyro[0]);
  EXPECT_DOUBLE_EQ(0, data.gyro[1]);
  EXPECT_DOUBLE_EQ(0, data.gyro[2]);
  EXPECT_DOUBLE_EQ(0, data.acc[0]);
  EXPECT_DOUBLE_EQ(0, data.acc[1]);
  EXPECT_DOUBLE_EQ(0, data.acc[2]);
  EXPECT_EQ(0, data.latency_us);
  EXPECT_EQ(0, data.counter);
}
