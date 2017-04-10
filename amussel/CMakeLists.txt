cmake_minimum_required(VERSION 2.8.3)
project(amussel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    auv_msgs 
    roscpp
    sensor_msgs
    snippets
    dynamic_reconfigure 
    cmake_modules)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
find_package(xacro)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
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

## Generate actions in the 'action' folder
#add_action_files(
#    FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   auv_msgs
#)

###################################################
## Dynamic reconfigure
###################################################
#generate_dynamic_reconfigure_options(cfg/ThrusterMapping.cfg cfg/DPSettings.cfg)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES pladypos
   CATKIN_DEPENDS auv_msgs message_runtime roscpp dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(pladypos
#   src/${PROJECT_NAME}/pladypos.cpp
# )

## Declare a cpp executable
#add_executable(pladypos_node src/pladypos_node.cpp src/PlaDyPosNode.cpp)
#add_executable(pladypos_node_v2 src/PlaDyPosNode_v2.cpp)

#add_executable(uros_logger_pladypos_node src/uros_logger.cpp)  
#add_executable(pladypos_node_v3 src/PlaDyPosNode_v3.cpp)
#add_executable(buddy_node_v1 src/BuddyTest_v1.cpp)
#add_executable(pro4_node_v1 src/VRTest_v1.cpp)     
#add_executable(nav_node src/nav_node.cpp)

## Add dependencies to the executable
#add_dependencies(pladypos_node ${PROJECT_NAME}_gencpp)
#if (DEFINED catkin_EXPORTED_TARGETS)
# add_dependencies(pladypos_node_v3 ${catkin_EXPORTED_TARGETS})
# add_dependencies(buddy_node_v1 ${catkin_EXPORTED_TARGETS})
# add_dependencies(pladypos_node_v2 ${catkin_EXPORTED_TARGETS})
# add_dependencies(pladypos_node ${catkin_EXPORTED_TARGETS})
# add_dependencies(pro4_node_v1 ${catkin_EXPORTED_TARGETS})
# add_dependencies(buddy_node_v1 ${catkin_EXPORTED_TARGETS})
# add_dependencies(uros_logger_pladypos_node ${catkin_EXPORTED_TARGETS})
#endif()

## Specify libraries to link a library or executable target against
#target_link_libraries(pladypos_node ${catkin_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
#target_link_libraries(pladypos_node_v2 ${catkin_LIBRARIES})
#target_link_libraries(pladypos_node_v3 ${catkin_LIBRARIES})

#target_link_libraries(uros_logger_pladypos_node ${catkin_LIBRARIES})
#target_link_libraries(buddy_node_v1 ${catkin_LIBRARIES})
#target_link_libraries(pro4_node_v1 ${catkin_LIBRARIES}) 

## Convert xacro models to urdf
if (xacro_FOUND)
 set(model_files "")
 file(GLOB model_xacro ${CMAKE_CURRENT_SOURCE_DIR}/data/models/xacro/*.urdf.xacro)
 set(expanded_directory ${CMAKE_CURRENT_SOURCE_DIR}/data/models/urdf/)
 execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${expanded_directory})
 foreach(it ${model_xacro})
   get_filename_component(basepath ${it} PATH)
   get_filename_component(basename ${it} NAME_WE)  
   set(expanded_file "${basepath}/../urdf/${basename}.urdf")  
   xacro_add_xacro_file(${it} ${expanded_file})
   set(model_files ${model_files} ${expanded_file})
 endforeach(it)
 add_custom_target(${PROJECT_NAME}_models ALL DEPENDS ${model_files})
endif()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pladypos pladypos_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pladypos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
