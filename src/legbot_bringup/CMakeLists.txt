cmake_minimum_required(VERSION 3.16)
project(legbot_bringup)
find_package(ament_cmake REQUIRED)
find_package(legbot_runtime REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
foreach(n reference_path_transform fastlio_odom_adapter cloud_transform)
add_executable(${n} src/${n}.cpp)
target_compile_features(${n} PUBLIC cxx_std_17)
ament_target_dependencies(${n} legbot_runtime tf2_sensor_msgs)
install(TARGETS ${n} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS
  scripts/go2_command_adapter
  scripts/go2_demo_sequencer
  scripts/fastlio_truth_monitor
  scripts/scan_waypoint_mission
  scripts/stair_cloud_sim
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models worlds rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
