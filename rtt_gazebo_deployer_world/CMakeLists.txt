cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_deployer_world)

## Find catkin macros and  braries
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo REQUIRED)
message(STATUS "GAZEBO version: ${GAZEBO_VERSION}") 
message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(Protobuf REQUIRED)

# this must be done in the beginning to adjust the cmake path
find_package(RSC 0.13 REQUIRED)
message(STATUS "RSC version: ${RSC_VERSION}")
include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS})
add_definitions(${RSC_DEFINITIONS})
link_directories(${RSC_LIBRARY_DIR})

find_package(RSB 0.13 REQUIRED)	
message(STATUS "RSB version: ${RSB_VERSION}")	
include_directories(BEFORE SYSTEM ${RSB_INCLUDE_DIRS})	
add_definitions(${RSB_DEFINITIONS})
	
find_package(RST COMPONENTS sandbox REQUIRED)
message(STATUS "RST version: ${RST_VERSION}")	
include_directories(BEFORE SYSTEM ${RST_INCLUDE_DIRS} ${RSTSANDBOX_INCLUDE_DIRS})	
add_definitions(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PROTUBUF_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

## Gazebo RTT plugin
add_definitions(-DRTT_COMPONENT)
add_library(${PROJECT_NAME} 
  src/gazebo_deployer_world_plugin.cpp
  src/gazebo_deployer_world_plugin.h
  src/RTTComponentPack.cpp
  src/RTTComponentPack.h)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -DRTT_STATIC)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTUBUF_LIBRARIES}
  ${RSB_LIBRARIES}	
  ${RST_LIBRARIES} 
)

set(CMAKE_BUILD_TYPE Debug)

orocos_generate_package(
    DEPENDS rtt_gazebo_msgs
)
