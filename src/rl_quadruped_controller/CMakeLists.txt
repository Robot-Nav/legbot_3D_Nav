cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
project(rl_quadruped_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)

find_package(yaml-cpp REQUIRED)

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
        HINTS $ENV{ONNXRUNTIME_ROOT}
        PATH_SUFFIXES include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
        HINTS $ENV{ONNXRUNTIME_ROOT}
        PATH_SUFFIXES lib)
if (NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "ONNX Runtime not found; set ONNXRUNTIME_ROOT")
endif ()

set(dependencies
        pluginlib
        rcpputils
        controller_interface
        realtime_tools
        control_input_msgs
        kdl_parser
        ament_index_cpp
        controller_common
)

# find dependencies
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/RlQuadrupedController.cpp

        src/common/ObservationBuffer.cpp

        src/FSM/StateFixedStand.cpp
        src/FSM/StateRL.cpp

        src/control/LowPassFilter.cpp
        src/control/Estimator.cpp

        src/robot/QuadrupedRobot.cpp
        src/robot/RobotLeg.cpp

)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        ${YAML_CPP_INCLUDE_DIR}
        ${ONNXRUNTIME_INCLUDE_DIR}
        src)
target_link_libraries(${PROJECT_NAME} PUBLIC
        "${TORCH_LIBRARIES}"
        "${ONNXRUNTIME_LIBRARY}"
        yaml-cpp
)
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${dependencies}
)

pluginlib_export_plugin_description_file(controller_interface rl_quadruped_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

add_executable(policy_contract_check test/policy_contract_check.cpp src/common/ObservationBuffer.cpp)
target_include_directories(policy_contract_check PRIVATE include/rl_quadruped_controller/common src/common)
target_link_libraries(policy_contract_check "${TORCH_LIBRARIES}" yaml-cpp)
install(TARGETS policy_contract_check DESTINATION lib/${PROJECT_NAME})

add_executable(onnx_policy_contract_check test/onnx_policy_contract_check.cpp)
target_include_directories(onnx_policy_contract_check PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(onnx_policy_contract_check "${ONNXRUNTIME_LIBRARY}" yaml-cpp)
install(TARGETS onnx_policy_contract_check DESTINATION lib/${PROJECT_NAME})
ament_package()
