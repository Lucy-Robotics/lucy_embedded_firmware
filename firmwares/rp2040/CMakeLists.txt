cmake_minimum_required(VERSION 3.22)
project(lucy_embedded_firmware_rp2040 NONE)

find_package(ament_cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/lucy_cargo.cmake)

lucy_cargo_firmware(BIN lucy-embedded-firmware-rp2040 TARGET thumbv6m-none-eabi UF2)

ament_package()
