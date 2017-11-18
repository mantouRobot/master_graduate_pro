cmake_minimum_required(VERSION 2.8.3)
project(usm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)
find_package(Pangolin REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH /home/mantou/chai3d-3.2.0/build)
set(CMAKE_MODULE_PATH /home/mantou/chai3d-3.2.0/build/extras/GLFW)
find_package(CHAI3D REQUIRED) # 放入了CHAI3DConfig.cmake文件
find_package(GLFW REQUIRED)
message(STATUS ${CHAI3D_LIBRARIES})

#find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui )

set(CMAKE_CXX_FLAGS "-std=c++11")

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usm
#  CATKIN_DEPENDS roscpp serial
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

#include_directories(/usr/include/eigen3)
include_directories(
  include
  ${Pangolin_INCLUDE_DIRS}
#  /usr/include/qt4
#  /usr/include/qt4/QtOpenGL/qgl.h
  ${catkin_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${CHAI3D_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(usm
#   src/${PROJECT_NAME}/usm.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(usm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(test_serial_motor src/test_serial_motor.cc)
#add_dependencies(test_serial_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_serial_motor ${catkin_LIBRARIES})

# 包含额外的链接库路径
link_directories("/home/mantou/usm_ws/libs")
link_directories(${CHAI3D_LIBRARY_DIRS})

add_executable(main src/main.cpp
                    src/main_class.cpp
                    src/force_sensor.cpp
                    src/ae210_force_driver.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${Pangolin_LIBRARIES}
#  ${GLUT_LIBRARIES}
  ${OpenCV_LIBS}
  libbiodaq.so)

#add_executable(test_gl src/test_gl.cpp)
#add_dependencies(test_gl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_gl ${catkin_LIBRARIES} ${Pangolin_LIBRARIES} ${GLUT_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_daq src/test_daq.cpp)
target_link_libraries(test_daq libbiodaq.so)

add_executable(test_step src/test_step.cpp)
target_link_libraries(test_step ${catkin_LIBRARIES} libbiodaq.so)

add_executable(test_follow src/test_follow.cpp)
target_link_libraries(test_follow ${catkin_LIBRARIES} libbiodaq.so)

add_executable(test_ms src/test_ms.cpp)
target_link_libraries(test_ms ${catkin_LIBRARIES} libbiodaq.so)

#link_libraries("/home/mantou/chai3d-3.2.0/extras/GLFW/lib/release/lin-x86_64-cc/libglfw.a")
#link_libraries("/home/mantou/chai3d-3.2.0/lib/release/lin-x86_64-cc/libchai3d.a")
#add_executable(test_chai3d src/test_chai3d.cpp src/MyDevice.cpp)
#target_link_libraries(test_chai3d ${catkin_LIBRARIES} ${CHAI3D_LIBRARIES} ${GLFW_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_mouse_move src/test_mouse_move.cpp)
target_link_libraries(test_mouse_move ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS usm usm_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_usm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
