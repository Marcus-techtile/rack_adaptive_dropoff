cmake_minimum_required(VERSION 3.0.2)
project(pallet_docking_xsquare)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/purePursuitReconfig.cfg
  cfg/quinticPlannerReconfig.cfg
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pallet_docking
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(utils_lib src/utils.cpp )
target_link_libraries(utils_lib ${catkin_LIBRARIES})

add_executable(docking_planner src/quintic_planner.cpp src/docking_planner_node.cpp)
target_link_libraries(docking_planner ${catkin_LIBRARIES} utils_lib)
add_dependencies(docking_planner ${PROJECT_NAME}_gencfg)

add_executable(docking_control src/docking_control_node.cpp src/fuzzy_control.cpp src/pure_pursuit_control.cpp)
target_link_libraries(docking_control ${catkin_LIBRARIES} utils_lib)
add_dependencies(docking_control ${PROJECT_NAME}_gencfg)

# add_executable(sideshift_control src/sideshift_control.cpp)
# target_link_libraries(sideshift_control ${catkin_LIBRARIES})
install(TARGETS utils_lib
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS docking_planner docking_control
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


