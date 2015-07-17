#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

// based on the ros tutorial on transforming opencv images to Image messages

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("camera", 1);

    // provider can be an url (e.g.: rtsp://10.0.0.1:554) or a number of device, (e.g.: 0 would be /dev/video0)
    std::string video_stream_provider;
    cv::VideoCapture cap;
    if (_nh.getParam("video_stream_provider", video_stream_provider)){
        ROS_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider);
        // If we are given a string of 4 chars or less (I don't think we'll have more than 100 video devices connected)
        // treat is as a number and act accordingly so we open up the videoNUMBER device
        if (video_stream_provider.size() < 4){
            ROS_INFO_STREAM("Getting video from provider: /dev/video" << video_stream_provider);
            cap.open(atoi(video_stream_provider.c_str()));
        }
        else{
            ROS_INFO_STREAM("Getting video from provider: " << video_stream_provider);
            cap.open(video_stream_provider);
        }
    }
    else{
        ROS_ERROR("Failed to get param 'video_stream_provider'");
        return -1;
    }

    std::string camera_name;
    _nh.param("camera_name", camera_name, std::string("camera"));
    ROS_INFO_STREAM("Camera name: " << camera_name);

    int fps;
    _nh.param("fps", fps, 240);
    ROS_INFO_STREAM("Throttling to fps: " << fps);

    std::string frame_id;
    _nh.param("frame_id", frame_id, std::string("camera"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);

    std::string camera_info_url;
    _nh.param("camera_info_url", camera_info_url, std::string(""));
    ROS_INFO_STREAM("Provided camera_info_url: '" << camera_info_url << "'");

    if(!cap.isOpened()){
        ROS_ERROR_STREAM("Could not open the stream.");
        return -1;
    }

    ROS_INFO_STREAM("Opened the stream, starting to publish.");

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;

    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
    // Get the saved camera info if any and setup some values if needed
    cam_info_msg = cam_info_manager.getCameraInfo();
    header.frame_id = frame_id;
    cam_info_msg.header = header;
    // If there is no distorsion model, add the most common one as sensor_msgs/CameraInfo says
    if (cam_info_msg.distortion_model == ""){
        cam_info_msg.distortion_model = "plumb_bob";
        // Don't let D matrix be empty then
        if (cam_info_msg.D.size() != 5)
            cam_info_msg.D.resize(5, 0.0);
    }
    cam_info_manager.setCameraInfo(cam_info_msg);

    ros::Rate r(fps);
    while (nh.ok()) {
        cap >> frame;
        if (pub.getNumSubscribers() > 0){
            // Check if grabbed frame is actually full with some content
            if(!frame.empty()) {
                msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                // Fill width and height if they are not set
                if (cam_info_msg.height == 0 || cam_info_msg.width == 0){
                    cam_info_msg.height = msg->height;
                    cam_info_msg.width = msg->width;
                }
                // The timestamps are in sync thanks to this publisher
                pub.publish(*msg, cam_info_msg, ros::Time::now());
            }

            ros::spinOnce();
        }
        r.sleep();
    }
}
