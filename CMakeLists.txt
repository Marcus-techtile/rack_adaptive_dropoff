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
  pallet_dock_msgs
  mbf_msgs
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
add_dependencies(utils_lib
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})
                
add_library(quintic_planner src/quintic_planner.cpp )
target_link_libraries(quintic_planner utils_lib ${catkin_LIBRARIES})
add_dependencies(quintic_planner
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(fuzzy_control src/fuzzy_control.cpp )
target_link_libraries(fuzzy_control utils_lib ${catkin_LIBRARIES})
add_dependencies(fuzzy_control
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(pure_pursuit_control src/pure_pursuit_control.cpp )
target_link_libraries(pure_pursuit_control utils_lib ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_control
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(docking_control src/docking_control.cpp )
target_link_libraries(docking_control  fuzzy_control 
                                       pure_pursuit_control ${catkin_LIBRARIES})
add_dependencies(docking_control
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(docking_planner src/docking_planner.cpp )
target_link_libraries(docking_planner quintic_planner docking_control ${catkin_LIBRARIES})
add_dependencies(docking_planner
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${PROJECT_NAME}_gencfg
                ${catkin_EXPORTED_TARGETS})

add_library(adaptive_docking_local_planner src/adaptive_docking_local_planner.cpp )
target_link_libraries(adaptive_docking_local_planner docking_planner ${catkin_LIBRARIES})
add_dependencies(adaptive_docking_local_planner
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})


# add_executable(adaptive_docking_node  src/adaptive_docking_node.cpp)
# target_link_libraries(adaptive_docking_node quintic_planner
#                                             fuzzy_control 
#                                             pure_pursuit_control 
#                                             docking_control
#                                             docking_planner
#                                             adaptive_docking_local_planner
#                                             ${catkin_LIBRARIES})
# add_dependencies(adaptive_docking_node 
#                   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#                   ${catkin_EXPORTED_TARGETS})

install(TARGETS utils_lib quintic_planner 
                          fuzzy_control 
                          pure_pursuit_control 
                          docking_control
                          docking_planner
                          adaptive_docking_local_planner
                          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(TARGETS docking_planner
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


