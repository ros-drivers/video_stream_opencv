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
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera", 1);
  
  std::string stream_url;
  if (nh.getParam("stream_url", stream_url)){
  	ROS_INFO_STREAM("Getting video from url: " << stream_url.c_str());
  }else{
  	ROS_ERROR("Failed to get param 'stream_url'");
  }

  int fps;
  if (nh.getParam("fps", fps)){
    ROS_INFO_STREAM("Throttling to fps: " << fps);
  }else{
    ROS_ERROR("Failed to get param 'fps'");
  }

  
  cv::VideoCapture cap(stream_url);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate r(fps);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }

    ros::spinOnce();
    r.sleep();
  }
}
