cmake_minimum_required(VERSION 3.5)
project(publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  src
  ${PCL__DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Qt5 settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Add executable
add_executable(publisher 
  src/main.cpp 
)

# Link libraries
target_link_libraries(publisher
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)

ament_target_dependencies(publisher
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
)

# Install executable
install(TARGETS publisher
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY src/
  DESTINATION src/)

ament_package()

