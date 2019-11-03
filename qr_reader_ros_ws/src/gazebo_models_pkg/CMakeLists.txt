cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package( PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED )
find_package( OGRE REQUIRED )

find_package( roscpp REQUIRED )



pkg_check_modules( EIGEN3 REQUIRED eigen3 )

FIND_PACKAGE( Boost 1.40 COMPONENTS system thread REQUIRED )
pkg_check_modules(GAZEBO REQUIRED gazebo>=6.0)
pkg_check_modules(SDF sdformat)
add_definitions(-std=c++11)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(rotors_description_dir "${CMAKE_SOURCE_DIR}/models/rotors_description")
set(scripts_dir "${CMAKE_SOURCE_DIR}/scripts")
add_custom_target(sdf
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND rm -f ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
	COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} 
	COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_SOURCE_DIR}/models/iris/iris.sdf
	COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
	)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs /opt/ros/$ENV{ROS_DISTRO}/lib)
add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_moving_platform SHARED src/gazebo_moving_platform.cpp)
target_link_libraries(gazebo_moving_platform mav_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE})
add_dependencies(gazebo_moving_platform mav_msgs)



