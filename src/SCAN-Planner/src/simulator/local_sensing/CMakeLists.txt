PROJECT(local_sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
# SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

option(USE_GPU "Build GPU OpenGL renderer" OFF)

  find_package(Eigen3 REQUIRED)
  find_package(OpenCV REQUIRED)
  # find_package(Pangolin 0.4 REQUIRED)

  if(USE_GPU)
    find_package(GLEW REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 QUIET)
    if(NOT glfw3_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr")
      set(glfw3_FOUND TRUE)
      set(glfw3_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr/include")
      set(glfw3_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/usr/lib/x86_64-linux-gnu/libglfw.so.3.3")
    endif()
    if(glfw3_FOUND)
      set(_glfw_header_found FALSE)
      if(EXISTS "/usr/include/GLFW/glfw3.h")
        set(_glfw_header_found TRUE)
      endif()
      foreach(_glfw_include_dir ${glfw3_INCLUDE_DIRS})
        if(EXISTS "${_glfw_include_dir}/GLFW/glfw3.h")
          set(_glfw_header_found TRUE)
        endif()
      endforeach()
      if(glfw3_LIBS AND NOT glfw3_LIBS STREQUAL "glfw" AND NOT EXISTS "${glfw3_LIBS}")
        set(_glfw_header_found FALSE)
      endif()
      if(NOT _glfw_header_found)
        set(glfw3_FOUND FALSE)
        set(glfw3_INCLUDE_DIRS "")
        set(glfw3_LIBS "")
      endif()
    endif()
    if(NOT glfw3_FOUND)
      message(FATAL_ERROR "USE_GPU=ON requires glfw3. Install libglfw3-dev or provide third_party/glfw.")
    endif()
  endif()
  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs cv_bridge)

      FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
      if(Boost_FOUND)
        INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
        LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
      endif()

      set(CMAKE_CXX_STANDARD_REQUIRED ON)
      set(CMAKE_CXX_EXTENSIONS ON)
      set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
      link_directories(${CMAKE_SOURCE_DIR}/lib)

  catkin_package(
      DEPENDS EIGEN3
      CATKIN_DEPENDS roscpp roslib pcl_ros sensor_msgs geometry_msgs nav_msgs cv_bridge
  )

# add_compile_options(-pg)
# SET(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

  include_directories(
    SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
  )

  

  add_executable(
    pcl_render_node
    src/pointcloud_render_node.cpp
    include/ikd-Tree/ikd_Tree.cpp
    include/FOV_Checker/FOV_Checker.cpp
    #src/raycast.cpp
  )

  target_link_libraries( pcl_render_node
    #include
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
  )

  if(USE_GPU)
    include_directories(
      SYSTEM
      ${OPENGL_INCLUDE_DIR}
      ${GLEW_INCLUDE_DIRS}
      ${glfw3_INCLUDE_DIRS}
    )

    add_library(GLAD include/glad.c)
    set(LIBS ${LIBS} GLAD)

    add_executable(
      opengl_render_node
      src/opengl_render_node.cpp
      include/ikd-Tree/ikd_Tree.cpp
      include/FOV_Checker/FOV_Checker.cpp
    )
    if(NOT glfw3_LIBS)
      set(glfw3_LIBS glfw)
    endif()
    target_link_libraries( opengl_render_node
      # include
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS}
      # pango_display
      ${Pangolin_LIBRARIES}
      ${Boost_LIBRARIES}
      ${OPENGL_LIBRARIES}
      ${GLEW_LIBRARIES}
      ${glfw3_LIBS}
      ${LIBS}
    )
  else()
    message(STATUS "USE_GPU=OFF: skipping opengl_render_node")
  endif()
