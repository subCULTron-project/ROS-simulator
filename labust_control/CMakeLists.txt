cmake_minimum_required(VERSION 2.8.3)
project(labust_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  auv_msgs
  navcon_msgs
  sensor_msgs
  dynamic_reconfigure
  roscpp
  snippets
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(rosdoc_lite)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)


## Generate actions
#add_action_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   actionlib_msgs
#)

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
   INCLUDE_DIRS include
   LIBRARIES pidtoolkit2
   CATKIN_DEPENDS auv_msgs roscpp snippets navcon_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include src/esctoolkit)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(hlmanager src/hl_node.cpp src/HLManager.cpp
#	include/labust/control/HLManager.hpp)
## Declare a cpp library
add_library(pidtoolkit2
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PSatDController.c
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c
)

set_source_files_properties(
   src/pidtoolkit/PIDBase.c
   src/pidtoolkit/PIFFController.c
   src/pidtoolkit/PSatDController.c 
   src/pidtoolkit/IPFFController.c
   src/pidtoolkit/PIDFFController.c
   src/pidtoolkit/IPDFFController.c PROPERTIES LANGUAGE CXX)

add_executable(manual_control src/manual_control/manual_control.cpp)
 
add_executable(fadp_controller2 src/FADPControl.cpp)
add_executable(fadp_3d_controller src/FADP_3DControl.cpp)
add_executable(hdg_controller2 src/HDGControl.cpp)
#add_executable(hdg_controller_direct src/HDGControlDirect.cpp)
add_executable(alt_controller2 src/ALTControl.cpp)
add_executable(depth_controller2 src/DepthControl.cpp)
add_executable(pitch_controller src/PitchControl.cpp)
add_executable(ualf_controller src/line_following/UALFControl.cpp)
add_executable(nu_manual src/NuManual.cpp)
add_executable(ref_manual src/RefManual.cpp)
add_executable(async_merger src/AsyncMerger.cpp)
add_executable(velocity_control	src/VelocityControl.cpp)

# ESC executables
add_executable(esc_uv src/esc/ESCControlClassic_UV.cpp src/esctoolkit/esc_classic.cpp)
add_executable(esc_ekf_uv src/esc/ESControlEKF_UV.cpp src/esctoolkit/esc_ekf_grad.cpp)
add_executable(esc_ekf_model_uv src/esc/ESControlEKF_MODEL_UV.cpp src/esctoolkit/esc_ekf_grad_model.cpp)
add_executable(esc_bounded_uv src/esc/ESControlBounded_UV.cpp src/esctoolkit/esc_bounded.cpp)


## Declare a cpp executable
# add_executable(labust_control_node src/labust_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(labust_control_node labust_control_generate_messages_cpp)
if (DEFINED catkin_EXPORTED_TARGETS)
 add_dependencies(manual_control ${catkin_EXPORTED_TARGETS})
 add_dependencies(fadp_controller2 ${catkin_EXPORTED_TARGETS})
 add_dependencies(fadp_3d_controller ${catkin_EXPORTED_TARGETS})
 add_dependencies(ualf_controller ${catkin_EXPORTED_TARGETS})
 add_dependencies(hdg_controller2 ${catkin_EXPORTED_TARGETS})
 #add_dependencies(hdg_controller_direct ${catkin_EXPORTED_TARGETS})
 add_dependencies(alt_controller2 ${catkin_EXPORTED_TARGETS})
 add_dependencies(depth_controller2 ${catkin_EXPORTED_TARGETS})
 add_dependencies(pitch_controller ${catkin_EXPORTED_TARGETS})
 add_dependencies(nu_manual ${catkin_EXPORTED_TARGETS})
 add_dependencies(ref_manual ${catkin_EXPORTED_TARGETS})
 add_dependencies(async_merger ${catkin_EXPORTED_TARGETS})
 add_dependencies(velocity_control ${catkin_EXPORTED_TARGETS})

 add_dependencies(esc_uv ${catkin_EXPORTED_TARGETS})
 add_dependencies(esc_ekf_uv ${catkin_EXPORTED_TARGETS})
 add_dependencies(esc_ekf_model_uv ${catkin_EXPORTED_TARGETS}) 
 add_dependencies(esc_bounded_uv ${catkin_EXPORTED_TARGETS}) 
 
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(manual_control ${catkin_LIBRARIES})
target_link_libraries(nu_manual ${catkin_LIBRARIES})
target_link_libraries(ref_manual ${catkin_LIBRARIES})
target_link_libraries(async_merger ${catkin_LIBRARIES})

target_link_libraries(ualf_controller
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(fadp_controller2
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(fadp_3d_controller
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(alt_controller2
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(depth_controller2
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(pitch_controller
  pidtoolkit2
  ${catkin_LIBRARIES}
)

target_link_libraries(hdg_controller2
  pidtoolkit2
  ${catkin_LIBRARIES}
)

#target_link_libraries(hdg_controller_direct
#  pidtoolkit2
#  ${catkin_LIBRARIES}
#)

target_link_libraries(velocity_control
  pidtoolkit2
  ${catkin_LIBRARIES}
)


target_link_libraries(esc_uv  ${catkin_LIBRARIES})
target_link_libraries(esc_ekf_uv  ${catkin_LIBRARIES})
target_link_libraries(esc_ekf_model_uv  ${catkin_LIBRARIES})
target_link_libraries(esc_bounded_uv  ${catkin_LIBRARIES})


###################
## Documentation ##
###################
if (BUILD_ROS_DOCS)
    if (rosdoc_lite_FOUND)
     set(doc_files mainpage.dox)   
     add_custom_target(${PROJECT_NAME}_docs ALL rosdoc_lite -o 
         ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR} DEPENDS ${doc_files})
     endif()
endif()

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
# install(TARGETS labust_control labust_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_labust_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
