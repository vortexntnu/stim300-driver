//
// Created by andreas on 21.03.19.
//

#include <iostream>
#include "serial_libserial.h"


SerialLibSerial::SerialLibSerial(const std::string &serial_port_name):serial_port_(serial_port_name)
{
  /*
  serial_port_.SetBaudRate(SerialPort::BAUD_921600);
  serial_port_.SetCharSize(SerialPort::CHAR_SIZE_8);
  serial_port_.SetFlowControl(SerialPort::FLOW_CONTROL_NONE);
  serial_port_.SetNumOfStopBits(SerialPort::STOP_BITS_1);
  serial_port_.SetParity(SerialPort::PARITY_NONE);
  */
}

void SerialLibSerial::open(BAUDRATE baudrate)
{
  SerialPort::BaudRate baud_rate;
  try
  {
  switch (baudrate)
  {
    case BAUDRATE::BAUD_4800 : baud_rate = SerialPort::BAUD_4800; break;
    case BAUDRATE::BAUD_9600 : baud_rate = SerialPort::BAUD_9600; break;
    case BAUDRATE::BAUD_19200 : baud_rate = SerialPort::BAUD_19200; break;
    case BAUDRATE::BAUD_38400 : baud_rate = SerialPort::BAUD_38400; break;
    case BAUDRATE::BAUD_57600 : baud_rate = SerialPort::BAUD_57600; break;
    case BAUDRATE::BAUD_115200 : baud_rate = SerialPort::BAUD_115200; break;
    case BAUDRATE ::BAUD_921600 : baud_rate = SerialPort::BAUD_921600; break;
    default: throw SerialPort::UnsupportedBaudRate("Not yet implemented baud rate.");
  }



    serial_port_.Open(
        baud_rate,
        SerialPort::CHAR_SIZE_8,
        SerialPort::PARITY_NONE,
        SerialPort::STOP_BITS_1,
        SerialPort::FLOW_CONTROL_NONE);

  } catch (SerialPort::AlreadyOpen &e){
    std::cerr<<e.what()<<"\n";
    return;
  } catch (SerialPort::OpenFailed &e){
    std::cerr<<e.what()<<"\n";
    return;
  } catch (SerialPort::UnsupportedBaudRate &e){
    std::cerr<<e.what()<<"\n";
    return;
  }catch (std::invalid_argument &e){
    std::cerr<<e.what()<<"\n";
    return;
  }
}

void SerialLibSerial::close()
{
  serial_port_.Close();
}

SerialLibSerial::~SerialLibSerial()
{
  serial_port_.Close();
}

void SerialLibSerial::readChunk(std::vector<unsigned char>& data_buffer, unsigned int num_of_bytes,
                                unsigned int ms_timeout)
{
  //std::vector<unsigned char> data_buffer;

  try
  {
    serial_port_.Read(data_buffer,num_of_bytes,ms_timeout);
  }
  catch (SerialPort::NotOpen& e)
  {
    std::cerr << e.what() << "\n";
    return;
  }
  catch (SerialPort::ReadTimeout& e)
  {
    //std::cerr<<"Read time out with "<<num_of_bytes - data_buffer.size()<<" remaining bytes to readChunk."<<std::endl;
    return;//throw e;
  }
  catch (std::runtime_error& e)
  {
    std::cerr << e.what() << "\n";
    return;
  }
}
/**
 * Read a single byte from the serial port. If no data is
 * available in the specified number of milliseconds (msTimeout),
 * then this method will throw ReadTimeout exception. If msTimeout
 * is 0, then this method will block till data is available.
 */
//unsigned char
//ReadByte( const unsigned int msTimeout = 0 )
//throw( NotOpen,
//ReadTimeout,
//std::runtime_error ) ;
bool SerialLibSerial::readByte(uint8_t & byte,unsigned int ms_timeout)
{
  try
  {
    byte = serial_port_.ReadByte(ms_timeout);
    return true;
  }
  catch (SerialPort::NotOpen& e)
  {
    std::cerr << e.what() << "\n";
    return false;
  }
  catch (SerialPort::ReadTimeout& e)
  {
    //std::cerr<<"Read time out with "<<num_of_bytes - data_buffer.size()<<" remaining bytes to readChunk."<<std::endl;
    return false;//throw e;
  }
  catch (std::runtime_error& e)
  {
    std::cerr <<"RUNTIME ERROR: "<< e.what() << "\n";
    return false;
  }
}


void SerialLibSerial::write(const SerialPort::DataBuffer &data_buffer)
{
  try
  {
    serial_port_.Write(data_buffer);
  } catch (SerialPort::NotOpen &e) {
    std::cerr<<e.what()<<"\n";
    return;
  } catch (std::runtime_error &e) {
    std::cerr<<e.what()<<"\n";
    return;
  }
}

bool SerialLibSerial::isDataAvailable()
{
  try
  {
    return serial_port_.IsDataAvailable();
  } catch (SerialPort::NotOpen &e) {
    std::cerr<<e.what()<<"\n";
    return false;
  } catch (std::runtime_error &e) {
    std::cerr<<e.what()<<"\n";
    return false;
  }

}