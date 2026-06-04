cmake_minimum_required(VERSION 2.8.3)
project(udp_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        mavros_msgs
        sensor_msgs
        nav_msgs
        roslib
        rosbag
        swarm_msgs
        rosfmt
        )

catkin_package(
        CATKIN_DEPENDS mavros_msgs sensor_msgs std_msgs swarm_msgs
)

include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(udp_online
        src/udp_online.cpp
        )
target_link_libraries(udp_online
        ${catkin_LIBRARIES}
        )

install(TARGETS udp_online
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch mesh images
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
