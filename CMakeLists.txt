cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv)
add_definitions(
    -std=c++11
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  nodelet
  dynamic_reconfigure
)


find_package(OpenCV)

generate_dynamic_reconfigure_options(
  cfg/VideoStream.cfg)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/video_stream.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(video_stream_node src/video_stream_node.cpp)
target_link_libraries(video_stream_node ${catkin_LIBRARIES})
set_target_properties(video_stream_node PROPERTIES OUTPUT_NAME video_stream)

install(TARGETS video_stream_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/test_video_resource.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_download_test_data(small.mp4
    http://techslides.com/demos/sample-videos/small.mp4
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 a3ac7ddabb263c2d00b73e8177d15c8d)
  add_rostest(test/test_video_file.test DEPENDENCIES small.mp4)
  add_rostest(test/test_mjpg_stream.test DEPENDENCIES small.mp4)
  add_rostest(test/test_rtsp_stream.test)
endif()
