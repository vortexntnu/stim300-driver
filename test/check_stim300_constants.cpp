#include "stim300_driver/stim300_constants.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace stim_const;

std::vector<DatagramIdentifier> DATAGRAM_IDS{
    DatagramIdentifier::CONFIGURATION_CRLF,
    DatagramIdentifier::CONFIGURATION,
    DatagramIdentifier::RATE,
    DatagramIdentifier::RATE_ACC,
    DatagramIdentifier::RATE_INCL,
    DatagramIdentifier::RATE_ACC_INCL,
    DatagramIdentifier::RATE_TEMP,
    DatagramIdentifier::RATE_ACC_TEMP,
    DatagramIdentifier::RATE_INCL_TEMP,
    DatagramIdentifier::RATE_ACC_INCL_TEMP,
    DatagramIdentifier::RATE_AUX,
    DatagramIdentifier::RATE_ACC_AUX,
    DatagramIdentifier::RATE_INCL_AUX,
    DatagramIdentifier::RATE_ACC_INCL_AUX,
    DatagramIdentifier::RATE_TEMP_AUX,
    DatagramIdentifier::RATE_ACC_TEMP_AUX,
    DatagramIdentifier::RATE_INCL_TEMP_AUX,
    DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX};

TEST(Stim300Constants, number_of_padding_bytes) {
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::CONFIGURATION_CRLF));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::CONFIGURATION));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE));
  EXPECT_EQ(0, number_of_padding_bytes(DatagramIdentifier::RATE_ACC));
  EXPECT_EQ(0, number_of_padding_bytes(DatagramIdentifier::RATE_INCL));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_INCL));
  EXPECT_EQ(3, number_of_padding_bytes(DatagramIdentifier::RATE_TEMP));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_TEMP));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_INCL_TEMP));
  EXPECT_EQ(1, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_INCL_TEMP));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_AUX));
  EXPECT_EQ(0, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_AUX));
  EXPECT_EQ(0, number_of_padding_bytes(DatagramIdentifier::RATE_INCL_AUX));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_INCL_AUX));
  EXPECT_EQ(3, number_of_padding_bytes(DatagramIdentifier::RATE_TEMP_AUX));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_TEMP_AUX));
  EXPECT_EQ(2, number_of_padding_bytes(DatagramIdentifier::RATE_INCL_TEMP_AUX));
  EXPECT_EQ(
      1, number_of_padding_bytes(DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX));
}

TEST(Stim300Constants, datagramIdentifier) {
  for (auto data_id : DATAGRAM_IDS)
    EXPECT_EQ(data_id,
              raw_to_datagram_identifier(datagram_identifier_to_raw(data_id)));
  // EXPECT_THROW(raw_to_datagram_identifier(0x00), std::out_of_range);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
