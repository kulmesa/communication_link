cmake_minimum_required(VERSION 3.5)
project(roswrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


#add_executable(gowrapper_pub src/wrapper_pub.cpp)
#ament_target_dependencies(gowrapper_pub rclcpp std_msgs)
#target_include_directories(gowrapper_pub PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#add_executable(gowrapper_sub src/wrapper_sub.cpp)
#ament_target_dependencies(gowrapper_sub rclcpp std_msgs px4_msgs)
#target_include_directories(gowrapper_sub PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_library(gowrapper SHARED src/wrapper_pub.cpp 
      src/wrapper_sub.cpp
      src/wrapper_init.cpp
      src/generic_subscription.cpp
      src/generic_publisher.cpp
      src/rosbag2_node.cpp
      src/qos.cpp
      src/typesupport_helpers.cpp
)
ament_target_dependencies(gowrapper rclcpp std_msgs px4_msgs ament_index_cpp)
target_include_directories(gowrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  gowrapper
#  gowrapper_pub
#  gowrapper_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
