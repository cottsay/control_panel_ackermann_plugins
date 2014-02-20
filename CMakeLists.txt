cmake_minimum_required(VERSION 2.8.3)
project(control_panel_ackermann_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ackermann_msgs control_panel joy_to_ackermann roscpp)

## System dependencies are found with CMake's conventions
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cpjoy_to_ackermann
#  CATKIN_DEPENDS ackermann_msgs control_panel joy_to_ackermann roscpp
#  DEPENDS Qt4
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

QT4_WRAP_UI(QT_CPJOY_TO_ACKERMANN_UI_H
  ui/CPJoyToAckermann.ui
)
QT4_WRAP_CPP(QT_CPJOY_TO_ACKERMANN_MOC_H
  include/cpjoy_to_ackermann/CPJoyToAckermann.h
)
QT4_WRAP_UI(QT_CPKEYDRIVEACKERMANN_UI_H
  ui/CPKeyDriveAckermann.ui
)
QT4_WRAP_CPP(QT_CPKEYDRIVEACKERMANN_MOC_H
  include/cpkeydriveackermann/CPKeyDriveAckermann.h
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

## Declare a cpp library
add_library(cpjoy_to_ackermann
  src/CPJoyToAckermann.cpp
  ${QT_CPJOY_TO_ACKERMANN_MOC_H}
  ${QT_CPJOY_TO_ACKERMANN_UI_H}
)
add_library(cpkeydriveackermann
  src/CPKeyDriveAckermann.cpp
  ${QT_CPKEYDRIVEACKERMANN_MOC_H}
  ${QT_CPKEYDRIVEACKERMANN_UI_H}
)

## Declare a cpp executable
# add_executable(control_panel_ackermann_plugins_node src/control_panel_ackermann_plugins_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(control_panel_ackermann_plugins_node control_panel_ackermann_plugins_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cpjoy_to_ackermann
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
target_link_libraries(cpkeydriveackermann
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS cpjoy_to_ackermann cpkeydriveackermann
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  control_panel_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_control_panel_ackermann_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
