#cmake_minimum_required(VERSION 2.8.3)
project(bio_ik_kinematics_plugin)

#add_compile_options(-std=c++11 -O3)

#set(CMAKE_CXX_FLAGS "-std=c++0x")

#set(CMAKE_CXX_COMPILER "clang++")
#set(CMAKE_C_COMPILER "clang")

#set(CMAKE_CXX_COMPILER "g++-4.8")

#set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")
#set(CMAKE_C_COMPILER "/usr/bin/gcc-4.8")


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
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")  

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++98")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wc++0x-compat")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -pedantic-errors")

catkin_add_gtest(utest test/utest.cpp)

message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(bio_ik_kinematics_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bio_ik_kinematics_plugin
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
)











