^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package video_stream_opencv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.6 (2020-12-08)
------------------
* #67 Deal better with looping on a video file and the start and stop frame selection
* #77 Noticed the default camera info was actually wrong, corrected
* #69 Get canonical file for target device and give a warning if we convert to a canonical path
* Fix typo when moving log to debug
* #57 Enable to select the start and stop points for playing video file.
* Avoid the excess of `start_frame` compared to `stop_frame` for playing video file.
* #55 Add title and travis badge
* #54 Enable testing on melodic
* Add linesep for each mp jpeg
* #53 Upgrade to OpenCV 4
* #52 video_stream: fix disconnect callbacks never being called
* Video_stream: make sure to always increase suscriber count
* #50 Add option to set output_encoding
* Add link to cv_bridge::cvtColor
* Add config to set output_encoding
* #49 Use variable config instead of class properties
* #51 install with source permissions, install small.mp4, install test directory to enable testing in catkin install space
* #45 libvideo_stream_opencv now depends on gencfg
* #44 Don't download test data on source directory
* #43 Add option to re-open camera device on read failure
* #39 Add dependency for gencfg
* #37 Support camera property settings on dynamic reconfigure
* #36 Fix bugs for thread initialization/deinitialization
* #34 Support changing parameter by using dynamic reconfigure
* Add dependencies for test
* Support dynamic_reconfigure
* #33 Convert video_stream node to nodelet, use nodelet for video_stream
* #31 Do not flip old image multiple times, closes #30
* #35 Enable CI
* Contributors: Alexander Rössler, GITAI, Leonardo Lai, Michael Sobrepera, Ryohei Ueda, Sam Pfeiffer, Sammy Pfeiffer, Yuki Furuta, moju zhao

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
