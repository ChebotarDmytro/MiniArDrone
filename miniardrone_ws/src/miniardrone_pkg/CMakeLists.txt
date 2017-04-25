cmake_minimum_required(VERSION 2.8.3)
project(miniardrone_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
#  roslint
)

add_library(netutil SHARED IMPORTED)
set_property(TARGET netutil PROPERTY
             IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/bluez/lib/libbluetooth.so)

# roslint
#set(ROSLINT_CPP_OPTS "--filter=-build/include")
#roslint_cpp(
#  src/main.cpp
##  src/ardrone_driver.cpp
##  include/miniardrone_pkg/ardrone_driver.h
#)


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
#  LIBRARIES miniardrone_pkg
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

#set(ARSDK_MANIFEST_HASH 40d5d08ff37a43b0b2deb641cc899d5356163db7)

#add_custom_target(ARSDK_MKDIR
#  COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/arsdk)

#add_custom_target(ARSDKBuildUtils
#  COMMENT "Building Parrot ARDroneSDK3"
#  WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/arsdk
#  COMMAND curl -s https://storage.googleapis.com/git-repo-downloads/repo > ./repo
#  COMMAND chmod a+x ./repo
#  COMMAND echo "y" | ./repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -b ${ARSDK_MANIFEST_HASH} -m release.xml
#  COMMAND ./repo sync
#  # TODO
#  COMMAND ./build.sh -p arsdk-native -t build-sdk -j || xterm -e ./build.sh -p arsdk-native -t build-sdk -j
#)
#add_dependencies(ARSDKBuildUtils ARSDK_MKDIR)
#set(ARDRONESDK3_PATH "${CATKIN_DEVEL_PREFIX}/arsdk/out/arsdk-native/staging/usr")

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  include
#  ${ARDRONESDK3_PATH}/include
#)

#link_directories(${ARDRONESDK3_PATH}/lib)

#add_library(libminiardrone src/main.cpp)
#target_link_libraries(libminiardrone
#  ${catkin_LIBRARIES}
#  arsal
#  arcontroller
#  ardiscovery
#)
#add_dependencies(libminiardrone ARSDKBuildUtils)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#set(SDK ${CATKIN_DEVEL_PREFIX}/arsdk/out/arsdk-native/staging/usr/include)
set(SDK /home/dima/WORKING_DIRECTORY/ArDroneSDK_3.0/out/arsdk-native/staging/usr/include)
set(BLE ${CATKIN_DEVEL_PREFIX}/bluez/include )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDK}
  ${BLE}
)

# Add headers directory
include_directories(include)


link_directories(/home/dima/WORKING_DIRECTORY/ArDroneSDK_3.0/out/arsdk-native/staging/usr/lib)


add_executable(main src/main.cpp)

add_dependencies(main
miniardrone_pkg_generate_messages_cpp)

target_link_libraries(main
  ${catkin_LIBRARIES}
  netutil
  arsal
  arcontroller
  ardiscovery
)

# Set SOURCE and HEADER files

#set(AA_HEADERS
#    include/miniardrone_pkg/ardrone_driver.h
#)

#set(AA_SOURCES
#    src/main.cpp
#)

# Build executable
#add_executable(main ${AA_SOURCES})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(miniardrone_pkg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(main  ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(miniardrone_pkg_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(main ${catkin_LIBRARIES} )
#target_link_libraries(main
#  ${catkin_LIBRARIES}
#  arsal
#  arcontroller
#  ardiscovery
#)


# roslint
#set(ROSLINT_CPP_OPTS "--filter=-runtime/references-runtime/threadsafe_fn")
#roslint_cpp(${AA_HEADERS} ${AA_SOURCES})

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
# install(TARGETS miniardrone_pkg miniardrone_pkg_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_miniardrone_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
