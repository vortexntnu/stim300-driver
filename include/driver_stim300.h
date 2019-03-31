//
// Created by andreas on 21.03.19.
//

#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include <boost/crc.hpp>
#include "../src/datagram_parser.h"
#include "../src/serial_libserial.h"
#include <array>
#include <vector>



class DriverStim300 {
public:

    explicit DriverStim300(SerialDriver &serial_driver);
    ~DriverStim300();
    double getAccX() const;
    double getAccY() const;
    double getAccZ() const;
    double getGyroX() const;
    double getGyroY() const;
    double getGyroZ() const;
    uint16_t getLatency_us() const;
    double getAverageTemp() const;
    bool isChecksumGood() const;
    bool isSensorStatusGood() const;
    uint8_t getInternalMeasurmentCounter() const;
    bool processPacket();

private:

    enum class Mode: uint8_t{Init, Normal, Service};
    Mode mode_;

    SerialDriver& serial_driver_;
    uint16_t serial_read_timeout_ms_;
    static constexpr uint8_t BUFFER_SIZE_{stim_300::MAX_DATAGRAM_SIZE*2};
    std::vector<uint8_t > buffer_;
    size_t n_read_bytes_;
    bool in_sync_;


    stim_300::DatagramIdentifier datagram_id_;
    stim_300::DatagramParser datagram_parser_;
    uint8_t datagram_size_;

    stim_300::SensorData sensor_data_;
    bool checksum_is_ok_;
    bool no_internal_error_;


    uint8_t crc_dummy_bytes_;
    bool verifyChecksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end, uint32_t& expected_CRC);

};


#endif //DRIVER_STIM300_DRIVER_STIM300_H
