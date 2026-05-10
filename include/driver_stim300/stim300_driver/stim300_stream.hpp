#pragma once

#include "stim300_driver/driver_stim300.hpp"
#include "stim300_driver/serial_unix.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

struct ImuMeasurement {
    double  acc_x,  acc_y,  acc_z;   // m/s²
    double  gyro_x, gyro_y, gyro_z;  // rad/s
    double  inc_x,  inc_y,  inc_z;
    int64_t stamp_ns;                 // nanoseconds since UNIX epoch
};

using MeasurementCallback = std::function<void(const ImuMeasurement &)>;
using StatusCallback      = std::function<void(Stim300Status, const DriverStim300 &)>;

class Stim300Stream {
public:
    Stim300Stream(
        const std::string & device,
        double              gravity,
        MeasurementCallback on_measurement,
        StatusCallback      on_status = nullptr);

    ~Stim300Stream();

    Stim300Stream(const Stim300Stream &)            = delete;
    Stim300Stream & operator=(const Stim300Stream &) = delete;

private:
    void read_loop();

    SerialUnix    serial_;
    DriverStim300 driver_;
    double        gravity_;

    MeasurementCallback on_measurement_;
    StatusCallback      on_status_;

    std::atomic<bool> running_{false};
    std::thread       thread_;
};
