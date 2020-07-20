cmake_minimum_required(VERSION 2.8.3)
project(mushr_mujoco_ros)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  ackermann_msgs
  sensor_msgs
  pybind11_catkin
)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AckermannDriveArray.msg
  BodyState.msg
  BodyStateArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  Rollout.srv
  Step.srv
  GetState.srv
  Reset.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ackermann_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

set(USE_GL 1) #USE_GL if we are to use GL with mujoco

if(NOT DEFINED $ENV{MUJOCO_LOCATION})
	set(MUJOCO_LOCATION $ENV{HOME}/.mujoco/mujoco200_linux)
else()
	set(MUJOCO_LOCATION $ENV{MUJOCO_LCOATION})
endif()

message(STATUS ${MUJOCO_LOCATION})

set(MUJOCO_INCLUDE_DIR ${MUJOCO_LOCATION}/include)
set(MUJOCO_BIN ${MUJOCO_LOCATION}/bin)

link_directories(${MUJOCO_BIN})

#Finding main mujoco library
if(${USE_GL})
file(GLOB LIB_MUJOCO ${MUJOCO_BIN}/libmujoco[0-9][0-9][0-9].so)
else()
file(GLOB LIB_MUJOCO ${MUJOCO_BIN}/libmujoco[0-9][0-9][0-9]nogl.so)
endif()
#Showing mujoco library found
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${MUJOCO_INCLUDE_DIR}
#  LIBRARIES mushr_mujoco_ros
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs ackermann_msgs sensor_msgs pybind11_catkin
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

message(STATUS ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIR}
  ${yaml-cpp_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mushr_mujoco_ros.cpp
# )

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

set(SOURCES
    src/body_ros_connector.cpp
    src/mjglobal.cpp
    src/mushr_mujoco_util.cpp
    src/mushr_ros_connector.cpp
    src/rollout.cpp
    src/sensors.cpp
    src/sim_record.cpp
    src/sim_state.cpp
    src/simple_viz.cpp
)

set(LIB
    include/body_ros_connector.h
    include/mjglobal.h
    include/mushr_mujoco_util.h
    include/mushr_ros_connector.h
    include/rollout.h
    include/sim_record.h
    include/sim_state.h
    include/simple_viz.h
    include/types.h
)

add_executable(${PROJECT_NAME}_node src/main.cpp ${SOURCES} ${LIB})
add_executable(${PROJECT_NAME}_block_buddy src/block_buddy_main.cpp ${SOURCES} ${LIB})
pybind_add_module(mushr_mujoco_py MODULE src/bindings.cpp ${SOURCES} ${LIB})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_block_buddy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mushr_mujoco_py ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
    target_link_libraries(${PROJECT_NAME}_block_buddy OpenMP::OpenMP_CXX)
    target_link_libraries(mushr_mujoco_py PRIVATE OpenMP::OpenMP_CXX)
endif()

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LIB_MUJOCO}
  ${yaml-cpp_LIBRARIES}
  GL
  GLU
  glut
  libglew.so
  libglfw.so
)

target_link_libraries(${PROJECT_NAME}_block_buddy
  ${catkin_LIBRARIES}
  ${LIB_MUJOCO}
  ${yaml-cpp_LIBRARIES}
  GL
  GLU
  glut
  libglew.so
  libglfw.so
)

target_link_libraries(mushr_mujoco_py PRIVATE
  ${catkin_LIBRARIES}
  ${LIB_MUJOCO}
  ${yaml-cpp_LIBRARIES}
  GL
  GLU
  glut
  libglew.so
  libglfw.so
)
