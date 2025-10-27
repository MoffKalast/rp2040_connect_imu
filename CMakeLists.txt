cmake_minimum_required(VERSION 3.10)
project(rp2040_connect_imu LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# ROS 2 runtime / build dependencies (you'll switch the code to rclcpp later)
find_package(rclcpp REQUIRED)          # keep here for when you convert
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
	include
	${Boost_INCLUDE_DIRS}
)

add_executable(rp2040_imu_node src/rp2040_imu_node.cpp)
ament_target_dependencies(rp2040_imu_node
	rclcpp
	sensor_msgs
	std_msgs
	geometry_msgs
	Boost
)
target_include_directories(rp2040_imu_node PRIVATE ${Boost_INCLUDE_DIRS})
install(TARGETS rp2040_imu_node
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install firmware into share (unchanged)
install(DIRECTORY firmware
	DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.launch" PATTERN "*.xml"
)

ament_package()
