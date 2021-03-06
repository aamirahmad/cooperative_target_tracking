cmake_minimum_required(VERSION 2.8.3)
project(tgt_detection)

find_package(cmake_modules REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#   visp_ros
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  tf
  pose_cov_ops
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

# add_message_files(
#   FILES
# )
# 
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs     
#   nav_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(tgt_detector src/detector.cpp)
target_link_libraries(tgt_detector ${catkin_LIBRARIES})