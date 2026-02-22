cmake_minimum_required(VERSION 3.8)
project(kb_coms_micro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kb_interfaces REQUIRED)

# Buscar la librería externa
add_subdirectory(../kb_serial_driver_lib build_serial)

set(dependencies
  rclcpp
  std_msgs
  kb_interfaces
)
include_directories(include)
include_directories(../kb_serial_driver_lib/include)

add_library(${PROJECT_NAME} SHARED
  src/kb_coms_micro/kb_coms_micro.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(KB_Coms_micro src/kb_coms_main.cpp)
ament_target_dependencies(KB_Coms_micro ${dependencies})
target_link_libraries(KB_Coms_micro ${PROJECT_NAME} serial_driver)

install(TARGETS
  ${PROJECT_NAME}
  KB_Coms_micro
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Agregar launch si se quiere poner launcher en este paquete
install(DIRECTORY DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
