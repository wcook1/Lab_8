cmake_minimum_required(VERSION 3.0.2)
project(ur3e_setup)

add_compile_options(-std=c++11)
add_compile_options(-lpthread)

find_package(catkin REQUIRED COMPONENTS	
	roscpp 
	moveit_msgs 
	moveit_core 
	moveit_ros_planning_interface
	moveit_ros_planning
)

catkin_package(
		CATKIN_DEPENDS
		roscpp moveit_msgs moveit_core moveit_ros_planning_interface moveit_ros_planning
)

include_directories(include ${catkin_INCLUDE_DIRS})
#-------------------------------------------------------------------------#

# Tactile Controller
add_executable(scene_setup src/scene_setup.cpp)
target_link_libraries(scene_setup ${catkin_LIBRARIES})