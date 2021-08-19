cmake_minimum_required(VERSION 3.0.2)
project(lazarillo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  urdf 
  rospy
  std_msgs 
  sensor_msgs
  message_generation
)

catkin_python_setup()

# Custom messages
add_message_files(
  FILES
  LlantasVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Build C++ nodes
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener ${catkin_EXPORTED_TARGETS})

# Python scripts
catkin_install_python(
  PROGRAMS 
  scripts/control_xbox.py 
  scripts/odom_pub.py
  scripts/a_star_server.py
  scripts/follow_path.py
  scripts/kinect_record.py
  scripts/kinect_play.py
  scripts/map_to_base.py
  scripts/initialize.py
  DESTINATION 
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)