cmake_minimum_required(VERSION 3.22)
project(lucy_embedded_firmware_linux NONE)

find_package(ament_cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/lucy_cargo.cmake)

lucy_cargo_firmware(BIN lucy-embedded-firmware-linux)

ament_package()
