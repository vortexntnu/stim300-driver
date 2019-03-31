//
// Created by andreas on 22.03.19.
//

#ifndef DRIVER_STIM300_SERIAL_DRIVER_H
#define DRIVER_STIM300_SERIAL_DRIVER_H

#include <vector>

class SerialDriver {

public:
    enum class BAUDRATE:uint32_t {
        BAUD_4800    = 4800,
        BAUD_9600    = 9600,
        BAUD_19200   = 19200,
        BAUD_38400   = 38400,
        BAUD_57600   = 57600,
        BAUD_115200  = 115200,
        BAUD_921600  = 921600
    };
    virtual void open(BAUDRATE baudrate) = 0;
    virtual void close() = 0;
    virtual bool readByte(uint8_t & byte,unsigned int ms_timeout) = 0;
    virtual void readChunk(std::vector<unsigned char> &data_buffer, unsigned int num_of_bytes, unsigned int ms_timeout) = 0;
    virtual void write(const std::vector< unsigned char > &data_buffer) = 0;
    virtual bool isDataAvailable() = 0;

    //class ReadTimeout : public std::runtime_error
    //{
    //public:
    //    explicit ReadTimeout(const std::string& whatArg [[maybe_unused]]): runtime_error(whatArg){}
    //} ;
    //class SerialPortNotOpen : public std::runtime_error
    //{
    //public:
    //    explicit SerialPortNotOpen(const std::string& whatArg [[maybe_unused]]): runtime_error(whatArg){}
    //} ;
};





#endif //DRIVER_STIM300_SERIAL_DRIVER_H
