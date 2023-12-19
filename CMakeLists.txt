cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(exploration)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(octomap_DIR "/opt/ros/noetic/share/octomap")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)

find_package (PCL 1.10 REQUIRED)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10 /usr/include/eigen3/) 

include_directories(${PCL_INCLUDE_DIRS})


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("OpenMP enabled")
endif()


find_package(catkin REQUIRED COMPONENTS
  #nlopt
  # ewok_optimization
  # ewok_poly_spline
  # ewok_ring_buffer
  geometry_msgs
  octomap_msgs
  octomap_ros
  mav_msgs
  mav_planning_msgs
  std_srvs
  roscpp
  pcl_ros
  tf_conversions
  std_msgs
  move_base_msgs
  actionlib
  message_generation
)

 add_message_files(
   FILES
   Frontiers.msg
   Frontier.msg
 )



 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs 
 )


catkin_package(
  INCLUDE_DIRS include 
 CATKIN_DEPENDS 
 message_runtime 
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    $(OCTOMAP_INCLUDE_DIRS)
    # ${NLOPT_LIBRARIES}
    include/
)

 add_library(${PROJECT_NAME}
   src/exploration.cpp
   src/frontier_selection.cpp
   src/vis_next.cpp
   # src/exploration_cmd.cpp
 )


add_executable(exploration_node src/exploration_node.cpp)
 target_link_libraries(
   exploration_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
 )
 add_dependencies(exploration_node ${catkin_EXPORTED_TARGETS} my_package_generate_messages_cpp)
 add_executable(frontier_selection_node src/frontier_selection_node.cpp)
 target_link_libraries(
   frontier_selection_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
 )

  add_executable(vis_next src/vis_next.cpp)
 target_link_libraries(
   vis_next
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
 )
add_dependencies(frontier_selection_node ${catkin_EXPORTED_TARGETS} my_package_generate_messages_cpp)
# add_executable(exploration_cmd_node src/exploration_cmd_node.cpp)
# target_link_libraries(
#   exploration_cmd_node
#   ${CHOLMOD_LIBRARY}
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${NLOPT_LIBRARIES}
# )
