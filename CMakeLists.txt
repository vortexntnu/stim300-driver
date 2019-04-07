cmake_minimum_required(VERSION 3.13)
project(driver_stim300)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)


#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to find_package(Threads).
#
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)


add_library(DriverStim300
        src/driver_stim300.cpp
        src/datagram_parser.cpp
        src/serial_unix.cpp)

target_compile_features(DriverStim300 PRIVATE cxx_std_17)

target_include_directories(DriverStim300
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )
target_link_libraries(DriverStim300 PRIVATE ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


add_executable(driver_stim300_example src/main.cpp)
target_link_libraries(driver_stim300_example PRIVATE DriverStim300)
