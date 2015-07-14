A package to view video streams based on the OpenCV VideoCapture module.

===

Based on the ROS [tutorial to convert opencv images to ROS messages](http://wiki.ros.org/image_transport/Tutorials/PublishingImages).

Example usages in launch folder:

    <launch>
       <!-- launch video stream -->
       <include file="$(find video_stream_opencv)/launch/camera.launch" >
            <!-- node name and ros graph name -->
            <arg name="camera_name" value="webcam" />
            <!-- means video device 0, /dev/video0, can be a video stream or a video file -->
            <arg name="video_stream_provider" value="0" />
            <!-- throttling the querying of frames to -->
            <arg name="fps" value="30" />
            <!-- visualize on an image_view window the stream generated -->
            <arg name="visualize" value="true" />
       </include>
    </launch>


You can use any input that OpenCV on your system accepts, e.g.:

    1) Video devices that appear in linux as /dev/videoX, e.g.: USB webcams appearing as /dev/video0

    2) Video streamings, e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov

    3) Video files, anything you can play, e.g.: myvideo.avi

    4) Etc.

In the scripts folder you'll find `test_video_resource.py` which you can use to test if your system
installation can use this node to open your video stream (not using any ROS). Just do:

    ./test_video_resource.py 0
    ./test_video_resource.py rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov
    ./test_video_resource.py /home/youruser/myvideo.mkv

And you'll see an output like:

    Trying to open resource: /dev/video0
    Correctly opened resource, starting to show feed.

With an OpenCV image show window showing the stream (which should close when pressing ESC, or Control+C the shell).


TODO: Add (optional) camera info.
