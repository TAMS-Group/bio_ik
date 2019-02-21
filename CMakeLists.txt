cmake_minimum_required(VERSION 2.8.12)
project(bio_ik)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  tf2
  tf2_eigen
  tf2_kdl
  tf2_geometry_msgs
)

find_package(OpenMP)
# the specific flag is not yet present in cmake 2.8.12
if(OpenMP_CXX_FOUND OR OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  add_compile_options(${OpenMP_CXX_FLAGS})
  if(NOT OpenMP_CXX_LIBRARIES)
    # cmake 2.8.12 does not yet specify the library - assume we might need libgomp
    set(OpenMP_LIBS gomp)
  else()
    set(OpenMP_LIBS ${OpenMP_CXX_LIBRARIES})
  endif()
else()
    message(WARNING "OPENMP NOT FOUND. You will suffer performance loss.")
    set(OpenMP_LIBS)
endif()

option(USE_FANN "build the neural-network-based IK solver (experimental)" OFF)
if(USE_FANN)
    find_library(FANN_LIBRARIES NAMES fann)
    find_path(FANN_INCLUDE_DIRS NAMES fann.h)
    if(NOT FANN_INCLUDE_DIRS OR NOT FANN_LIBRARIES)
        message(FATAL_ERROR "Neural network solver requested, but libfann was not found.")
    else()
        message("Found libfann: ${FANN_LIBRARIES} / ${FANN_INCLUDE_DIRS}")
    endif()
else()
    set(FANN_LIBRARIES)
    set(FANN_INCLUDE_DIRS)
endif()

option(USE_CPPOPTLIB "Include gradient-based solvers from CppNumericalSolvers (bio_ik also provides its own solver)" OFF)
if(USE_CPPOPTLIB)
    find_path(CPPOPTLIB_INCLUDE_DIRS
        NAMES cppoptlib/solver/bfgssolver.h
        HINTS ../../CppNumericalSolvers/include)
    if(NOT CPPOPTLIB_INCLUDE_DIRS)
        message(FATAL_ERROR "cppoptlib support requested, but the headers could not be found.")
    else()
        message("Found cppoptlib: ${CPPOPTLIB_INCLUDE_DIRS}")
    endif()
    add_definitions(-DENABLE_CPP_OPTLIB)
else()
    set(CPPOPTLIB_INCLUDE_DIRS)
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        eigen_conversions
        moveit_core
        moveit_ros_planning
        pluginlib
        roscpp
        tf2
        tf2_kdl
        tf2_geometry_msgs
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-frecord-gcc-switches)
endif()

add_compile_options(-std=c++11)

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ftree-vectorize>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

include_directories(
    include
    ${FANN_INCLUDE_DIRS}
    ${CPPOPTLIB_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/goal_types.cpp
    src/kinematics_plugin.cpp
    src/problem.cpp

    src/ik_test.cpp
    src/ik_gradient.cpp
    src/ik_evolution_1.cpp
    src/ik_evolution_2.cpp
)

if(USE_FANN)
    list(APPEND SOURCES src/ik_neural.cpp)
endif()

if(USE_CPPOPTLIB)
    list(APPEND SOURCES src/ik_cppoptlib.cpp)
endif()

add_library(${PROJECT_NAME} ${SOURCES})

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${FANN_LIBRARIES}
  ${OpenMP_LIBS}

  -static-libgcc
  -static-libstdc++
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES bio_ik_kinematics_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
