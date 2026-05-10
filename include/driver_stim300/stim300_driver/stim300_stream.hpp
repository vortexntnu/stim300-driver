#pragma once

#include "stim300_driver/driver_stim300.hpp"
#include "stim300_driver/serial_unix.hpp"

#include <atomic>
#include <functional>
#include <string>
#include <thread>

using MeasurementCallback = std::function<void(const DriverStim300 &)>;
using StatusCallback      = std::function<void(Stim300Status, const DriverStim300 &)>;

class Stim300Stream {
public:
    Stim300Stream(
        const std::string & device,
        MeasurementCallback on_measurement,
        StatusCallback      on_status = nullptr);

    ~Stim300Stream();

    Stim300Stream(const Stim300Stream &)            = delete;
    Stim300Stream & operator=(const Stim300Stream &) = delete;

private:
    void read_loop();

    SerialUnix    serial_;
    DriverStim300 driver_;

    MeasurementCallback on_measurement_;
    StatusCallback      on_status_;

    std::atomic<bool> running_{false};
    std::thread       thread_;
};
