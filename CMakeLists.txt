cmake_minimum_required(VERSION 3.15)
project(bio_ik)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
  tf2_eigen
  tf2_ros
  tf2_kdl
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-frecord-gcc-switches)
endif()

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ftree-vectorize>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

include_directories(
    include
    ${FANN_INCLUDE_DIRS}
    ${CPPOPTLIB_INCLUDE_DIRS}
)

set(SOURCES
    src/goal_types.cpp
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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  ${FANN_LIBRARIES}
  ${OpenMP_LIBS}

  -static-libgcc
  -static-libstdc++
)

add_library(${PROJECT_NAME}_plugin SHARED
  ${SOURCES}
  src/kinematics_plugin.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}_plugin
  PRIVATE
    ${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


pluginlib_export_plugin_description_file(
  moveit_core
  bio_ik_kinematics_description.xml
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bio_ik_test test/utest.cpp)
  target_link_libraries(bio_ik_test
    bio_ik
  )
  target_include_directories(bio_ik_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PUBLIC $<INSTALL_INTERFACE:include>)
  
endif()

ament_package()
