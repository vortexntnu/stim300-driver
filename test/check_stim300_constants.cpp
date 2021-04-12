#include "../src/stim300_constants.h"
#include <gtest/gtest.h>
#include <vector>

using namespace stim_const;

std::vector<DatagramIdentifier> DATAGRAM_IDS{ DatagramIdentifier::CONFIGURATION_CRLF,
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
                                              DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX };

TEST(Stim300Constants, numberOfPaddingBytes)
{
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::CONFIGURATION_CRLF));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::CONFIGURATION));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE));
  EXPECT_EQ(0, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC));
  EXPECT_EQ(0, numberOfPaddingBytes(DatagramIdentifier::RATE_INCL));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_INCL));
  EXPECT_EQ(3, numberOfPaddingBytes(DatagramIdentifier::RATE_TEMP));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_TEMP));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_INCL_TEMP));
  EXPECT_EQ(1, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_INCL_TEMP));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_AUX));
  EXPECT_EQ(0, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_AUX));
  EXPECT_EQ(0, numberOfPaddingBytes(DatagramIdentifier::RATE_INCL_AUX));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_INCL_AUX));
  EXPECT_EQ(3, numberOfPaddingBytes(DatagramIdentifier::RATE_TEMP_AUX));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_TEMP_AUX));
  EXPECT_EQ(2, numberOfPaddingBytes(DatagramIdentifier::RATE_INCL_TEMP_AUX));
  EXPECT_EQ(1, numberOfPaddingBytes(DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX));
}

TEST(Stim300Constants, datagramIdentifier)
{
  for (auto data_id : DATAGRAM_IDS)
    EXPECT_EQ(data_id, rawToDatagramIdentifier(datagramIdentifierToRaw(data_id)));
  //EXPECT_THROW(rawToDatagramIdentifier(0x00), std::out_of_range);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
