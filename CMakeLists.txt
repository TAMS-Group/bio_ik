project(bio_ik_kinematics_plugin)

set(ROS_BUILD_TYPE Release)

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
  tf_conversions
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
    DEPENDS OpenMP
    CATKIN_DEPENDS moveit_core moveit_ros_planning actionlib roscpp pluginlib tf tf_conversions
    INCLUDE_DIRS include
    LIBRARIES bio_ik_kinematics_plugin
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

FILE(GLOB bio_ik_source_files src/*.cpp)

message(STATUS "SOURCES ${bio_ik_source_files}")

add_library(bio_ik_kinematics_plugin ${bio_ik_source_files})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -ffast-math")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frecord-gcc-switches") 

message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(bio_ik_kinematics_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bio_ik_kinematics_plugin
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
  gcc_s
  gcc
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest test/utest.cpp)
    target_link_libraries(utest ${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS bio_ik_kinematics_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES bio_ik_kinematics_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
