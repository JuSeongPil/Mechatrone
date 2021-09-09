cmake_minimum_required(VERSION 3.0.2)
project(rohang2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf

  cv_bridge
  image_transport
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
   FILES
	GpsPoint.msg
	CameraPoint.msg

)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rohang2
  LIBRARIES opencv
  CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport rospy
  message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(rohang2 src/rohang2.cpp src/Function.cpp)
target_link_libraries(rohang2 ${catkin_LIBRARIES})

add_executable(tlqkf src/image_node.cpp)
target_link_libraries(tlqkf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(tlqkf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})















