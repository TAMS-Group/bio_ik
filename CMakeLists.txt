cmake_minimum_required(VERSION 3.10.2)
project(bio_ik)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
# find_package(eigen_conversions REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(MOVEIT_LIB_NAME moveit_bio_ik_kinematics_plugin)

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

set(THIS_PACKAGE_INCLUDE_DIRS
    bio_ik/include
)

pluginlib_export_plugin_description_file(moveit_core bio_ik_kinematics_plugin_description.xml)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-frecord-gcc-switches)
endif()

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ftree-vectorize>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

include_directories(include)

add_library(${MOVEIT_LIB_NAME} SHARED
    src/goal_types.cpp
    src/kinematics_plugin.cpp
    src/problem.cpp
    src/ik_evolution_1.cpp
    src/ik_evolution_2.cpp
)

target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

if(USE_FANN)
    list(APPEND THIS_PACKAGE_SOURCE_FILES src/ik_neural.cpp)
endif()

if(USE_CPPOPTLIB)
    list(APPEND THIS_PACKAGE_SOURCE_FILES src/ik_cppoptlib.cpp)
endif()

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MOVEIT_LIB_NAME}
  ${FANN_LIBRARIES}
  ${OpenMP_LIBS}

  -static-libgcc
  -static-libstdc++
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
    # eigen_conversions
    moveit_core
    moveit_ros_planning
    pluginlib
    rclcpp
    tf2
    tf2_kdl
    tf2_geometry_msgs
    EIGEN3
)

target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE "MOVEIT_TRAC_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

# # prevent pluginlib from using boost
# target_compile_definitions(${MOVEIT_LIB_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# # Causes the visibility macros to use dllexport rather than dllimport,
# # which is appropriate when building the dll but not consuming it.
# target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE "MOVEIT_BIO_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

ament_export_targets(${MOVEIT_LIB_NAME} HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${MOVEIT_LIB_NAME}
  EXPORT ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${MOVEIT_LIB_NAME})
ament_export_targets(${MOVEIT_LIB_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()