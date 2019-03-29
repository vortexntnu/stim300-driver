cmake_minimum_required(VERSION 3.13)
project(driver_stim300)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)

add_library(DatagramParser
        src/datagram_parser.cpp
        src/datagram_parser.h
        src/stim300_constants.h)
