cmake_minimum_required(VERSION 3.0.2)
project(rack_adaptive_dropoff)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  mbf_msgs
  rack_detection_msg
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
    include
  LIBRARIES
    utils_lib_dropoff
    quintic_planner_dropoff 
    fuzzy_control_dropoff 
    pure_pursuit_control_dropoff 
    docking_control_dropoff 
    docking_planner_dropoff 
    adaptive_dropoff_local_planner
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${rack_detection_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(utils_lib_dropoff src/utils.cpp )
target_link_libraries(utils_lib_dropoff ${catkin_LIBRARIES})
add_dependencies(utils_lib_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})
                
add_library(quintic_planner_dropoff src/quintic_planner.cpp )
target_link_libraries(quintic_planner_dropoff utils_lib_dropoff ${catkin_LIBRARIES})
add_dependencies(quintic_planner_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(fuzzy_control_dropoff src/controller/fuzzy_control.cpp )
target_link_libraries(fuzzy_control_dropoff utils_lib_dropoff ${catkin_LIBRARIES})
add_dependencies(fuzzy_control_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(pure_pursuit_control_dropoff src/controller/pure_pursuit_control.cpp )
target_link_libraries(pure_pursuit_control_dropoff utils_lib_dropoff ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_control_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(docking_control_dropoff src/docking_control.cpp src/filters)
target_link_libraries(docking_control_dropoff  fuzzy_control_dropoff 
                                        pure_pursuit_control_dropoff
                                       ${catkin_LIBRARIES})
add_dependencies(docking_control_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(docking_planner_dropoff src/docking_planner.cpp )
target_link_libraries(docking_planner_dropoff quintic_planner_dropoff docking_control_dropoff ${catkin_LIBRARIES})
add_dependencies(docking_planner_dropoff
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_library(adaptive_dropoff_local_planner src/adaptive_dropoff_local_planner.cpp )
target_link_libraries(adaptive_dropoff_local_planner docking_planner_dropoff ${catkin_LIBRARIES})
add_dependencies(adaptive_dropoff_local_planner
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

add_executable(adaptive_dropoff_node  src/adaptive_dropoff_node.cpp)
target_link_libraries(adaptive_dropoff_node 
                                            adaptive_dropoff_local_planner
                                            ${catkin_LIBRARIES})
add_dependencies(adaptive_dropoff_node 
                  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                  ${catkin_EXPORTED_TARGETS})

install(TARGETS utils_lib_dropoff 
                quintic_planner_dropoff 
                fuzzy_control_dropoff  
                pure_pursuit_control_dropoff 
                docking_control_dropoff 
                docking_planner_dropoff 
                adaptive_dropoff_local_planner
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(TARGETS docking_planner
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


