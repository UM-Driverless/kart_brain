cmake_minimum_required(VERSION 3.10)
project(kb_serial_driver_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(serial_driver SHARED
  src/kb_coms_serial_driver.cpp
)

target_include_directories(serial_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(serial_driver PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS serial_driver
  EXPORT serial_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(EXPORT serial_driver_targets
  FILE serial_driverConfig.cmake
  NAMESPACE serial_driver::
  DESTINATION lib/cmake/serial_driver
)
