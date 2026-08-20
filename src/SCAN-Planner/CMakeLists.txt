cmake_minimum_required(VERSION 3.0.2)
project(scan_planner)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

add_message_files(FILES Bspline.msg DataDisp.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs message_runtime nav_msgs sensor_msgs std_msgs visualization_msgs
)

# The upstream algorithm, tools and assets live together in this Catkin package.
# Building its sources in one target avoids importing the upstream EGO-derived
# duplicate package names (plan_env, bspline_opt, and others) into the workspace.
set(SCAN_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/planner)

include_directories(
  ${SCAN_SOURCE_ROOT}/plan_manage/include
  ${SCAN_SOURCE_ROOT}/plan_env/include
  ${SCAN_SOURCE_ROOT}/path_searching/include
  ${SCAN_SOURCE_ROOT}/bspline_opt/include
  ${SCAN_SOURCE_ROOT}/traj_utils/include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(scan_planner_node
  ${SCAN_SOURCE_ROOT}/plan_manage/src/scan_planner_node.cpp
  ${SCAN_SOURCE_ROOT}/plan_manage/src/scan_replan_fsm.cpp
  ${SCAN_SOURCE_ROOT}/plan_manage/src/planner_manager.cpp
  ${SCAN_SOURCE_ROOT}/plan_env/src/grid_map.cpp
  ${SCAN_SOURCE_ROOT}/plan_env/src/raycast.cpp
  ${SCAN_SOURCE_ROOT}/path_searching/src/dyn_a_star.cpp
  ${SCAN_SOURCE_ROOT}/bspline_opt/src/uniform_bspline.cpp
  ${SCAN_SOURCE_ROOT}/bspline_opt/src/bspline_optimizer.cpp
  ${SCAN_SOURCE_ROOT}/bspline_opt/src/gradient_descent_optimizer.cpp
  ${SCAN_SOURCE_ROOT}/traj_utils/src/planning_visualization.cpp
  ${SCAN_SOURCE_ROOT}/traj_utils/src/polynomial_traj.cpp
)
target_link_libraries(scan_planner_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(scan_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_closed_loop_controller
  ${SCAN_SOURCE_ROOT}/plan_manage/src/closed_loop_controller.cpp
  ${SCAN_SOURCE_ROOT}/bspline_opt/src/uniform_bspline.cpp
)
target_link_libraries(scan_closed_loop_controller ${catkin_LIBRARIES})
add_dependencies(scan_closed_loop_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_a1_cmd_adapter
  ${SCAN_SOURCE_ROOT}/plan_manage/src/a1_cmd_adapter.cpp
  ${SCAN_SOURCE_ROOT}/bspline_opt/src/uniform_bspline.cpp
)
target_link_libraries(scan_a1_cmd_adapter ${catkin_LIBRARIES})
add_dependencies(scan_a1_cmd_adapter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS scan_planner_node scan_closed_loop_controller scan_a1_cmd_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
