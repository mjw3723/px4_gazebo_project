cmake_minimum_required(VERSION 3.8)
project(px4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(offboard_node src/offboard_node.cpp)
ament_target_dependencies(offboard_node rclcpp px4_msgs geometry_msgs sensor_msgs cv_bridge std_msgs  nav_msgs)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node rclcpp px4_msgs geometry_msgs sensor_msgs  std_msgs tf2 tf2_ros nav_msgs)

add_executable(image_node src/image_node.cpp)
ament_target_dependencies(image_node rclcpp geometry_msgs sensor_msgs std_msgs cv_bridge)
target_link_libraries(image_node 
  ${OpenCV_LIBS}  
)
install(TARGETS
  offboard_node  
  sensor_node
  image_node 
  DESTINATION lib/${PROJECT_NAME}
)
# Python 설치
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  px4_control/mediapipe_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
