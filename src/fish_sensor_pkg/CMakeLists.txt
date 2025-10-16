cmake_minimum_required(VERSION 3.8)
project(fish_sensor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# I2C Device Library
add_library(i2c_device
  src/i2c_device.cpp
)
target_include_directories(i2c_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(i2c_device PUBLIC c_std_99 cxx_std_17)

# MPU6500 Library
add_library(mpu6500
  src/mpu6500.cpp
)
ament_target_dependencies(mpu6500
  "rclcpp"
)
target_include_directories(mpu6500 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpu6500 i2c_device)

# AS5600 Library
add_library(as5600
  src/as5600.cpp
)
target_include_directories(as5600 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(as5600 i2c_device)

# Madgwick Filter Library
add_library(madgwick_filter
  src/madgwick_filter.cpp
)
# ↓↓↓↓ このブロックが今回の修正点 ↓↓↓↓
ament_target_dependencies(madgwick_filter
  "sensor_msgs"
)
target_include_directories(madgwick_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Main Node
add_executable(fish_sensor_node src/fish_sensor_node.cpp)
ament_target_dependencies(fish_sensor_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(fish_sensor_node
  mpu6500
  as5600
  madgwick_filter
  i2c_device
)

# Install
install(TARGETS
  fish_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
)

ament_package()