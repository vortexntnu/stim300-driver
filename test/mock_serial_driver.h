
#ifndef DRIVER_STIM300_MOCK_SERIAL_DRIVER_H
#define DRIVER_STIM300_MOCK_SERIAL_DRIVER_H

#include "stim300_driver/stim300_constants.hpp"
#include <algorithm>
#include <boost/crc.hpp>
#include <vector>

class DatagramBuffer {
public:
  DatagramBuffer() {
    // Create a simple datagram with crc checksum
    datagram_.push_back(175);
    for (int i = 1; i < 63; ++i) {
      datagram_.push_back(0);
    }
    auto end = datagram_.cend();
    auto begin = datagram_.cbegin();
    uint8_t crc_dummy_bytes = stim_const::number_of_padding_bytes(
        stim_const::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX);
    boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false,
                                           false);
    std::vector<uint8_t> crc_buffer(end - begin - sizeof(uint32_t) +
                                    crc_dummy_bytes);
    std::copy(begin, end - sizeof(uint32_t), crc_buffer.begin());

    /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
    for (size_t i = 0; i < crc_dummy_bytes; ++i)
      crc_buffer[crc_buffer.size() - (1 + i)] = 0x00;

    crc_32_calculator.process_bytes(crc_buffer.data(), crc_buffer.size());
    uint32_t crc = crc_32_calculator.checksum();

    datagram_[59] = (crc & 0xff000000u) >> 24u;
    datagram_[60] = (crc & 0x00ff0000u) >> 16u;
    datagram_[61] = (crc & 0x0000ff00u) >> 8u;
    datagram_[62] = (crc & 0x000000ffu);

    it_ = datagram_.cbegin();
  }
  bool get_next_byte(uint8_t &byte) {
    byte = *it_++;
    return true;
  }

  const uint8_t *data() const { return datagram_.data(); }

private:
  std::vector<uint8_t>::const_iterator it_;
  std::vector<uint8_t> datagram_;
};

#endif // DRIVER_STIM300_MOCK_SERIAL_DRIVER_H
