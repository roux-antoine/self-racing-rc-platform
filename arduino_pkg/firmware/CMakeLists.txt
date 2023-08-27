cmake_minimum_required(VERSION 3.0.2)
project(firmware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

# some doc on that: https://github.com/queezythegreat/arduino-cmake/blob/master/cmake/Platform/Arduino.cmake
generate_arduino_firmware(aim
  SRCS aim.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/ttyACM1
  # SERIAL picocom /dev/ttyACM0  # found in https://github.com/ros-drivers/rosserial/issues/459, seems to work without
)
