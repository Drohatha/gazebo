cmake_minimum_required(VERSION 2.8.3)

project(gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	mavros_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(landOn src/landOnRoomba.cpp)
add_executable(roombaRepeater src/roombaRepeater)

target_link_libraries(landOn ${catkin_LIBRARIES})
target_link_libraries(roombaRepeater ${catkin_LIBRARIES})

