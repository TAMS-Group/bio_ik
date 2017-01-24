#cmake_minimum_required(VERSION 2.8.3)
project(bio_ik_kinematics_plugin)

#add_compile_options(-std=c++11 -O3)

#set(CMAKE_CXX_FLAGS "-std=c++0x")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  actionlib
  roscpp
  pluginlib
  std_srvs
  tf
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
    set(OpenMP_LIBS gomp)
else()
    message(STATUS "OPENMP NOT FOUND")
endif()

catkin_package(
    DEPENDS
        OpenMP
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(bio_ik_kinematics_plugin src/ikbio.cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x ${OpenMP_FLAGS}")

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(bio_ik_kinematics_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bio_ik_kinematics_plugin
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
)











