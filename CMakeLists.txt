cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

catkin_package()

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

if(0)
add_executable(video_stream src/video_stream.cpp)
target_link_libraries(video_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS video_stream
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif(0)

add_executable(main src/main.cpp)
target_link_libraries(main ${INCLUDE_LIBS})

install(PROGRAMS scripts/test_video_resource.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
