cmake_minimum_required(VERSION 3.8)
project(roby_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# Find libgpiod (system library, optional — not present on PC)
find_library(GPIOD_LIBRARY gpiod)
find_path(GPIOD_INCLUDE_DIR gpiod.h)

if(GPIOD_LIBRARY AND GPIOD_INCLUDE_DIR)
  set(HAS_GPIOD TRUE)
  message(STATUS "libgpiod found: ${GPIOD_LIBRARY}")
else()
  set(HAS_GPIOD FALSE)
  message(STATUS "libgpiod NOT found — building with mock GPIO only")
endif()

# Main library
add_library(${PROJECT_NAME} SHARED
  src/roby_system.cpp
  src/stepper_driver.cpp
  src/servo_driver.cpp
  src/safety_monitor.cpp
  src/encoder_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(HAS_GPIOD)
  target_include_directories(${PROJECT_NAME} PRIVATE ${GPIOD_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${GPIOD_LIBRARY})
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_GPIOD)
endif()

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface roby_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Manual hardware tests (only on Pi5 with gpiod)
if(HAS_GPIOD)
  add_executable(test_stepper_hw test/test_stepper_hw.cpp src/stepper_driver.cpp)
  target_include_directories(test_stepper_hw PUBLIC include ${GPIOD_INCLUDE_DIR})
  target_link_libraries(test_stepper_hw ${GPIOD_LIBRARY})
  target_compile_definitions(test_stepper_hw PRIVATE HAS_GPIOD)
  install(TARGETS test_stepper_hw DESTINATION lib/${PROJECT_NAME})

  add_executable(test_encoder_driver_hw test/test_encoder_driver_hw.cpp src/encoder_driver.cpp)
  target_include_directories(test_encoder_driver_hw PUBLIC include ${GPIOD_INCLUDE_DIR})
  target_link_libraries(test_encoder_driver_hw ${GPIOD_LIBRARY})
  target_compile_definitions(test_encoder_driver_hw PRIVATE HAS_GPIOD)
  install(TARGETS test_encoder_driver_hw DESTINATION lib/${PROJECT_NAME})
endif()

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_stepper_driver test/test_stepper_driver.cpp src/stepper_driver.cpp)
  target_include_directories(test_stepper_driver PUBLIC include)
  ament_target_dependencies(test_stepper_driver rclcpp)

  ament_add_gtest(test_servo_driver test/test_servo_driver.cpp src/servo_driver.cpp)
  target_include_directories(test_servo_driver PUBLIC include)
  ament_target_dependencies(test_servo_driver rclcpp)

  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp src/safety_monitor.cpp)
  target_include_directories(test_safety_monitor PUBLIC include)
  ament_target_dependencies(test_safety_monitor rclcpp)

  # PID closed-loop (header-only pid.hpp) — pur, sans hardware ni deps ROS
  ament_add_gtest(test_pid test/test_pid.cpp)
  target_include_directories(test_pid PUBLIC include)

  # Filtre rejet d'outliers + porte de sortie (header-only) — pur, sans hardware
  ament_add_gtest(test_outlier_filter test/test_outlier_filter.cpp)
  target_include_directories(test_outlier_filter PUBLIC include)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(ament_index_cpp hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)
ament_package()
