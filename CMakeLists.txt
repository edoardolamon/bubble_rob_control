cmake_minimum_required(VERSION 3.16)
project(bubble_rob_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)                 # use 20 on Lyrical
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)      # the static ZeroMQ client goes into a .so

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ZMQ_CLIENT_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/zmqRemoteApi/clients/cpp)
add_subdirectory(${ZMQ_CLIENT_DIR} ${CMAKE_BINARY_DIR}/zmq_client EXCLUDE_FROM_ALL)

# hardware plugin
add_library(bubble_rob_system SHARED src/bubble_rob_system.cpp)
target_include_directories(bubble_rob_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(bubble_rob_system
  PUBLIC hardware_interface::hardware_interface pluginlib::pluginlib
         rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE RemoteAPIClient)
target_include_directories(bubble_rob_system PRIVATE ${ZMQ_CLIENT_DIR})   # RemoteAPIClient.h
pluginlib_export_plugin_description_file(hardware_interface bubble_rob_control.xml)

# obstacle avoider node (Step 6)
add_executable(obstacle_avoider src/obstacle_avoider.cpp)
target_link_libraries(obstacle_avoider
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})

install(TARGETS bubble_rob_system LIBRARY DESTINATION lib)
install(TARGETS obstacle_avoider DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_package()