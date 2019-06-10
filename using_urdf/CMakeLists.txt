cmake_minimum_required(VERSION 2.8.3)
project(using_urdf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib 
  actionlib_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES communicator.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(lld src/lld.cpp)

add_executable(client src/client.cpp)
add_executable(hld src/hld_main.cpp src/hld.cpp)

add_executable(marker src/marker_publisher.cpp)
target_link_libraries(
  marker 
  ${catkin_LIBRARIES}
)

target_link_libraries(
  client
  ${catkin_LIBRARIES}
)

target_link_libraries( 
  hld
  ${catkin_LIBRARIES} lld
)

add_dependencies(
  client
  ${using_urdf_EXPORTED_TARGETS}
)

add_dependencies(
  hld
  ${using_urdf_EXPORTED_TARGETS}
)

add_dependencies(
  marker
  ${using_urdf_EXPORTED_TARGETS}
)


