set(DRIVER_LIB stim300_driver_lib)

add_library(${DRIVER_LIB}
    SHARED
)

target_sources(
    ${DRIVER_LIB}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/datagram_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_unix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_stim300.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stim300_stream.cpp
)

target_include_directories(
    ${DRIVER_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(
    TARGETS ${DRIVER_LIB}
    DESTINATION lib
)

