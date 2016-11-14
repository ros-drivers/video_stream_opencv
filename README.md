A package to view video streams based on the [OpenCV VideoCapture module](http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#videocapture), easy way to publish on a ROS Image topic (including camera info) usb cams, ethernet cameras, video streams or video files. It also supports flipping of images.

![Screenshot of the plugin working with a webcam, video stream and video file](https://raw.githubusercontent.com/pal-robotics/video_stream_opencv/master/screenshot_usage.png)

Example usages in launch folder (**only the argument `video_stream_provider` is mandatory**):

```xml
<launch>
   <!-- launch video stream -->
   <include file="$(find video_stream_opencv)/launch/camera.launch" >
        <!-- node name and ros graph name -->
        <arg name="camera_name" value="webcam" />
        <!-- means video device 0, /dev/video0 -->
        <arg name="video_stream_provider" value="0" />
        <!-- throttling the querying of frames to -->
        <arg name="fps" value="30" />
        <!-- setting frame_id -->
        <arg name="frame_id" value="webcam" />
        <!-- camera info loading, take care as it needs the "file:///" at the start , e.g.:
        "file:///$(find your_camera_package)/config/your_camera.yaml" -->
        <arg name="camera_info_url" value="" />
        <!-- flip the image horizontally (mirror it) -->
        <arg name="flip_horizontal" value="false" />
        <!-- flip the image vertically -->
        <arg name="flip_vertical" value="false" />
        <!-- force a width and height, 0 means no forcing -->
        <arg name="width" value="640"/>
        <arg name="height" value="480"/>
        <!-- visualize on an image_view window the stream generated -->
        <arg name="visualize" value="true" />
   </include>
</launch>
```

Based on the ROS [tutorial to convert opencv images to ROS messages](http://wiki.ros.org/image_transport/Tutorials/PublishingImages).

===

You can use any input that OpenCV on your system accepts, e.g.:

* Video devices that appear in linux as /dev/videoX, e.g.: USB webcams appearing as /dev/video0

* Video streamings, e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov

* Video files, anything you can play, e.g.: myvideo.avi

* Etc.

===

In the scripts folder you'll find `test_video_resource.py` which you can use to test if your system
installation can use this node to open your video stream (not using any ROS). Just do any of those:

    ./test_video_resource.py 0
    ./test_video_resource.py rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov
    ./test_video_resource.py /home/youruser/myvideo.mkv

And you'll see an output like:

    Trying to open resource: /dev/video0
    Correctly opened resource, starting to show feed.

With an OpenCV image show window showing the stream (which should close when pressing ESC, or Control+C the shell).

