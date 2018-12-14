^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package video_stream_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5 (2018-12-14)
------------------
* Exit the program if we reach the end of the video when playing a video file (Issue #23)
* Throw exception when a frame cant be captured (Issue #23, PR #27)
* Add loop_video parameter for videofiles (PR #24)
* Contributors: Sammy Pfeiffer, iory

1.1.4 (2018-07-23)
------------------
* Fix bug, cv::Mat needs to be cloned otherwise all entries in the queue will point to the same frame. This resulted in always returning the last received frame.
* Contributors: Axel13fr

1.1.3 (2018-07-11)
------------------
* Fixing ever growing memory when using boost::sync_queue (Issue #20) by reimplementing thread safety of frames queue using std classes. boost::sync_queue is buggy (keeps on allocating memory over time).
* Contributors: Axel13fr

1.1.2 (2018-06-12)
------------------
* Fix empty frame id in camera info header when providing a calibration file.
* Contributors: tim-fan

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
