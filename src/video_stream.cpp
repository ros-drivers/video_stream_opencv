#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
    image_transport::Publisher pub = it.advertise("camera", 1);

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
        return 1;
    }

    int fps;
    _nh.param("fps", fps, 240);
    ROS_INFO_STREAM("Throttling to fps: " << fps);

    if(!cap.isOpened()){
        ROS_ERROR_STREAM("Could not open the stream.");
        return 1;
    }

    ROS_INFO_STREAM("Opened the stream, starting to publish.");

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate r(fps);
    while (nh.ok()) {
        cap >> frame;
        if (pub.getNumSubscribers() > 0){
            // Check if grabbed frame is actually full with some content
            if(!frame.empty()) {
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                pub.publish(msg);
            }

            ros::spinOnce();
        }
        r.sleep();
    }
}
