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
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

add_executable(${PROJECT_NAME}
  src/heatmap_publisher.cpp
  src/main.cpp
 )

target_include_directories(${PROJECT_NAME} PUBLIC include/)

ament_target_dependencies(${PROJECT_NAME}
 rclcpp nav_msgs tf2 tf2_ros geometry_msgs tf2_geometry_msgs sensor_msgs OpenCV cv_bridge)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

