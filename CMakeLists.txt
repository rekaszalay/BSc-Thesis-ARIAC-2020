cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  geometry_msgs
  tf2_geometry_msgs
  rosgraph_msgs
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  tf2_ros
  tf2
  message_generation
)

find_package(PCL REQUIRED)

add_service_files(DIRECTORY srv
  FILES
  GetNextModel.srv
  GetPlacePosition.srv
)

generate_messages(
        DEPENDENCIES
        nist_gear
        std_msgs
        geometry_msgs
        rosgraph_msgs
        actionlib_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(controller 
	src/example_control_node.cpp
  src/utilities.cpp
  )
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(gantry_control_node
	src/gantry_control_node.cpp
  src/utilities.cpp
  )
add_dependencies(gantry_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gantry_control_node ${catkin_LIBRARIES})


## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  script/ariac_example_node.py
  script/tf2_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
 config/sample_gear_conf.yaml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
