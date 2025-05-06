cmake_minimum_required(VERSION 2.8.3)
project(rp2040_connect_imu)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
	sensor_msgs
	std_msgs
	geometry_msgs
    message_generation
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy geometry_msgs sensor_msgs std_msgs geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/imu.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)