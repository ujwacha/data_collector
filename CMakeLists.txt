cmake_minimum_required(VERSION 3.8)
project(Serial_Bridge_Skeleton)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)

# set(SERIAL_LIB "/usr/local/lib/libserial.so")
# set(SERIAL_INCLUDE_DIR "/usr/local/include/libserial")

include_directories(${SERIAL_INCLUDE_DIR})

# Add include directories

# add_library(data_publisher SHARED src/main.cpp)

add_executable(data_publisher src/main.cpp)
#target_compile_definitions(Serial_Skili PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(data_publisher nav_msgs sensor_msgs tf2_ros rclcpp std_msgs geometry_msgs rclcpp_components )


# rclcpp_components_register_node(
#   data_publisher 
#   PLUGIN "Serial_Bridge_Skeleton::Sayer"
#   EXECUTABLE Serial_Skili 
#   )



# ament_export_targets(export_data_publisher)



install(TARGETS
  data_publisher 
# EXPORT export_data_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
