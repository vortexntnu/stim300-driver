#include "stim300_driver/stim300_stream.hpp"

#include <chrono>

Stim300Stream::Stim300Stream(
    const std::string & device,
    double              gravity,
    MeasurementCallback on_measurement,
    StatusCallback      on_status)
: serial_(device, stim_const::BaudRate::BAUD_921600)
, driver_(serial_)
, gravity_(gravity)
, on_measurement_(std::move(on_measurement))
, on_status_(std::move(on_status))
{
    running_ = true;
    thread_  = std::thread(&Stim300Stream::read_loop, this);
}

Stim300Stream::~Stim300Stream()
{
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void Stim300Stream::read_loop()
{
    while (running_) {
        const auto status = driver_.update();
        switch (status) {
        case Stim300Status::NEW_MEASURMENT: {
            const auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            on_measurement_({
                .acc_x    = driver_.getAccX() * gravity_,
                .acc_y    = driver_.getAccY() * gravity_,
                .acc_z    = driver_.getAccZ() * gravity_,
                .gyro_x   = driver_.getGyroX(),
                .gyro_y   = driver_.getGyroY(),
                .gyro_z   = driver_.getGyroZ(),
                .inc_x    = driver_.getIncX(),
                .inc_y    = driver_.getIncY(),
                .inc_z    = driver_.getIncZ(),
                .stamp_ns = stamp_ns,
            });
            break;
        }
        case Stim300Status::NORMAL:
            break;
        default:
            if (on_status_)
                on_status_(status, driver_);
            break;
        }
    }
}
