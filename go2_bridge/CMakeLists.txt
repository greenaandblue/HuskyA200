cmake_minimum_required(VERSION 3.16)
project(go2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# unitree_sdk2 installed with: cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && sudo make install
set(UNITREE_PREFIX "/opt/unitree_robotics")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_PREFIX}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_bridge src/go2_bridge.cpp)
ament_target_dependencies(go2_bridge rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2)
target_link_libraries(go2_bridge unitree_sdk2)

# unitree_sdk2 bundles its own (newer) CycloneDDS, while ROS (Foxy especially) ships an
# older CycloneDDS in /opt/ros/<distro>/include. Put the SDK's headers FIRST so that
# dds/dds.h and ddscxx resolve to the SDK's copy, not ROS's.
target_include_directories(go2_bridge BEFORE PRIVATE
  ${UNITREE_PREFIX}/include
  ${UNITREE_PREFIX}/include/ddscxx)

# Same at runtime: load the SDK's libddsc/libddscxx, not ROS's. DT_RPATH (old-style) is
# searched before LD_LIBRARY_PATH, which sourcing ROS fills with /opt/ros/<distro>/lib.
set_target_properties(go2_bridge PROPERTIES
  BUILD_RPATH "${UNITREE_PREFIX}/lib"
  INSTALL_RPATH "${UNITREE_PREFIX}/lib")
target_link_options(go2_bridge PRIVATE "-Wl,--disable-new-dtags")

install(TARGETS go2_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
