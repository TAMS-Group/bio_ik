#cmake_minimum_required(VERSION 2.8.3)
project(bio_ik_kinematics_plugin)

set(ROS_BUILD_TYPE Release)

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

add_library(bio_ik_kinematics_plugin
    src/kinematics_plugin.cpp
    src/ik_base.cpp
    src/ik_cppoptlib.cpp
    src/ik_evolution_1.cpp
    src/ik_evolution_2.cpp
    src/ik_evolution_3.cpp
    src/ik_gradient.cpp
    src/ik_test.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

#set(CMAKE_CXX_FLAGS "-O0 ${CMAKE_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -ftree-vectorize -ffast-math -ftree-vectorizer-verbose=5")



#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -ffast-math -ftree-vectorizer-verbose=7")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -ffast-math")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frecord-gcc-switches") # readelf -p .GCC.command.line ...

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++98")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wc++0x-compat")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -pedantic-errors")

#catkin_add_gtest(utest test/utest.cpp)

message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}")

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(bio_ik_kinematics_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bio_ik_kinematics_plugin
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
  gcc_s
  gcc
  #fann
  #doublefann
)


#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(bio_ik_test test/utest.launch test/utest.cpp)
#  target_link_libraries(bio_ik_test ${catkin_LIBRARIES})
#endif()


if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest test/utest.cpp)
    target_link_libraries(utest ${PROJECT_NAME})
endif()


install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS bio_ik_kinematics_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES bio_ik_kinematics_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
