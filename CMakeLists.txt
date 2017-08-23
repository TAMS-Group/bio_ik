cmake_minimum_required(VERSION 2.8.12)
project(bio_ik)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
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
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        pluginlib
        roscpp
        tf
        tf_conversions
    DEPENDS
        OpenMP
)

add_compile_options(-frecord-gcc-switches)

add_compile_options(-std=c++11)
add_compile_options(${OpenMP_FLAGS})

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ftree-vectorize>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

# You can provide CppOptLib in a folder parallel to this one,
# to use it for gradient-based optimization.
# However, the gradient-based optimization implemented within this package
# was evaluated to be faster. So you probably don't want this anyway...
#add_definitions(-DENABLE_CPP_OPTLIB)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(bio_ik_SOURCES
    src/kinematics_plugin.cpp
    src/goal_types.cpp
    src/problem.cpp

    src/ik_test.cpp
    src/ik_gradient.cpp
    src/ik_evolution_1.cpp
    src/ik_evolution_2.cpp

#    src/ik_neural.cpp
#    src/ik_cppoptlib.cpp
)

add_library(${PROJECT_NAME} ${bio_ik_SOURCES})

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
  gcc_s
  gcc

#  fann
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest test/utest.cpp)
    target_link_libraries(utest ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES bio_ik_kinematics_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
