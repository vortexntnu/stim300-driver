#include "stim300_driver/stim300_stream.hpp"

Stim300Stream::Stim300Stream(
    const std::string & device,
    MeasurementCallback on_measurement,
    StatusCallback      on_status)
: serial_(device, stim_const::BaudRate::BAUD_921600)
, driver_(serial_)
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
        case Stim300Status::NEW_MEASURMENT:
            on_measurement_(driver_);
            break;
        case Stim300Status::NORMAL:
            break;
        default:
            if (on_status_)
                on_status_(status, driver_);
            break;
        }
    }
}
