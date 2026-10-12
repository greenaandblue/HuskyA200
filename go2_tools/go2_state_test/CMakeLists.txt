cmake_minimum_required(VERSION 3.16)
project(go2_state_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(UNITREE_PREFIX "/opt/unitree_robotics")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_PREFIX}")
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_state_test main.cpp)
target_link_libraries(go2_state_test unitree_sdk2)
target_include_directories(go2_state_test BEFORE PRIVATE
  ${UNITREE_PREFIX}/include ${UNITREE_PREFIX}/include/ddscxx)
# Always load the SDK's libddsc, even if ROS is sourced
set_target_properties(go2_state_test PROPERTIES BUILD_RPATH "${UNITREE_PREFIX}/lib")
target_link_options(go2_state_test PRIVATE "-Wl,--disable-new-dtags")
