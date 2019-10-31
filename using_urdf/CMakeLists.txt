cmake_minimum_required(VERSION 2.8.3)
project(using_urdf)

add_compile_options(-Wall -Wconversion -Wextra -std=c++14)
set(CMAKE_CXX_CPPCHECK "cppcheck" "–enable=all -Wall -Wextra -Wconversion --inline-suppr")
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
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
message_runtime
std_msgs
)

include_directories(
include 
${catkin_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS})

add_library(lld 
src/lld.cpp)
add_dependencies(lld using_urdf_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(client src/client.cpp src/pose.cpp)
add_dependencies(client using_urdf_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(hld src/hld_main.cpp src/hld.cpp)
add_dependencies(hld using_urdf_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(marker src/marker_publisher.cpp)
add_dependencies(marker using_urdf_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(demo src/publisher.cpp src/pose.cpp)
add_dependencies(demo using_urdf_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

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
target_link_libraries(
  demo 
  ${catkin_LIBRARIES}
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
add_dependencies(
  demo
  ${using_urdf_EXPORTED_TARGETS}
)


