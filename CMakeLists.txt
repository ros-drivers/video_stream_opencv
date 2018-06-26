cmake_minimum_required(VERSION 3.1)
project(video_stream_opencv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge camera_info_manager image_transport roscpp roslib
                 rospy sensor_msgs
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
include_directories(
  include
)
set(INCLUDE_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(video_stream src/video_stream.cpp)
target_link_libraries(video_stream ${INCLUDE_LIBS})

install(TARGETS video_stream
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/test_video_resource.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
