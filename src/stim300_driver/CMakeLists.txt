set(DRIVER_LIB stim300_driver_lib)

add_library(${DRIVER_LIB}
    SHARED
)

target_sources(
    ${DRIVER_LIB}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/datagram_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/driver_stim300.cpp
)

find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(${DRIVER_LIB} PUBLIC Boost::system)

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
