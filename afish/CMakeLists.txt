cmake_minimum_required(VERSION 2.8.3)
project(afish)

find_package(catkin REQUIRED COMPONENTS 
    auv_msgs 
    roscpp
    sensor_msgs
    snippets
    dynamic_reconfigure 
    cmake_modules
    message_generation
    rospy
    std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
find_package(xacro)

catkin_package(
   CATKIN_DEPENDS auv_msgs message_runtime roscpp dynamic_reconfigure message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

#xacro
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
