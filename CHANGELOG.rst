^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package video_stream_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2018-04-20)
------------------
* Fix error of using camera_name instead of identifying the type of the provider
  to check when the provider is a video file, and act accordingly in the producer thread
* Prevent locking when ROS SIGINT arrives and the image queue is empty.
* Added rate limiting to camera_fps_rate also if videofile is used as input
* Contributors: Andrea Ranieri, Avio, Sam Pfeiffer, Sammy Pfeiffer

1.1.0 (2018-03-23)
------------------
* Update to use thread to read.
  Update to new parameters.
  Update description.
* Fixed image flip bug, the flip implementation is inconsistent with OpenCV API.
* Contributors: Sammy Pfeiffer, Zihan Chen, kantengri

1.0.2 (2016-11-14)
------------------

1.0.1 (2016-11-14)
------------------
* Releasable version
* Contributors: Sammy Pfeiffer, Stefano Probst, Wiebe Van Ranst
