cmake_minimum_required(VERSION 3.8)
project(floor_heat_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)




add_executable(${PROJECT_NAME}
  src/map_publisher.cpp
  src/main.cpp
 )

target_include_directories(${PROJECT_NAME} PUBLIC include/)

ament_target_dependencies(${PROJECT_NAME}
 rclcpp nav_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

