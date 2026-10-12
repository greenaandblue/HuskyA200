cmake_minimum_required(VERSION 3.16)
project(go2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# unitree_sdk2 installed with: cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && sudo make install
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_bridge src/go2_bridge.cpp)
ament_target_dependencies(go2_bridge rclcpp geometry_msgs nav_msgs std_srvs tf2)
target_link_libraries(go2_bridge unitree_sdk2)

install(TARGETS go2_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
