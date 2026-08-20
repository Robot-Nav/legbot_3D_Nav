cmake_minimum_required(VERSION 3.0.2)
project(legbot_bringup)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp tf)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(reference_path_transform src/reference_path_transform.cpp)
target_link_libraries(reference_path_transform ${catkin_LIBRARIES})

add_executable(fastlio_odom_adapter src/fastlio_odom_adapter.cpp)
target_link_libraries(fastlio_odom_adapter ${catkin_LIBRARIES})

install(PROGRAMS
  scripts/pct_tomography
  scripts/pct_plan
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(TARGETS reference_path_transform fastlio_odom_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
