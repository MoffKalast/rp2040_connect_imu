cmake_minimum_required(VERSION 2.8.3)
project(rp2040_connect_imu)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
	std_msgs
	geometry_msgs
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(rp2040_imu_node src/rp2040_imu_node.cpp)
target_link_libraries(rp2040_imu_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)