cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_device src/video_device.cpp)
target_link_libraries(video_device ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(video_stream src/video_stream.cpp)
target_link_libraries(video_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
