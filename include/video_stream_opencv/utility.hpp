#ifndef _VIDEO_STREAM_OPENCV_UTILITY_HPP_
#define _VIDEO_STREAM_OPENCV_UTILITY_HPP_

#include <string>
#include <tuple>

#include <boost/pointer_cast.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <video_stream_opencv/image_capture.hpp>

namespace video_stream_opencv {
// Based on the ros tutorial on transforming opencv images to Image messages
sensor_msgs::CameraInfo get_default_camera_info_from_image_msg(const sensor_msgs::ImageConstPtr img) {
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header = img->header;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;

    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = {1.0, 0.0, img->width/2.0,
                      0.0, 0.0, img->height/2.0,
                      0.0, 0.0, 1.0};
    // Give a reasonable default rectification matrix
    cam_info_msg.R = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    // Give a reasonable default projection matrix
    cam_info_msg.P = {1.0, 0.0, img->width/2.0, 0.0,
                      0.0, 0.0, img->height/2.0, 0.0,
                      0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}

void capture_frames(ImageCapture& cap) {
    ros::Rate camera_fps_rate(cap.camera_fps);
    cv::Mat frame;

    // Read frames as fast as possible
    while (ros::ok()) {
        cap.read(frame);
        if (cap.isFile()) {
            camera_fps_rate.sleep();
        }
        if(!frame.empty()) {
            cap.push(frame.clone());
        }
    }
}

/**
 * Returns a tuple of form <flip_image, flip_value>
 * If flip_image:bool is false, using flip_value:int is undefined behavior
 *
 * From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode
 * FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
 */
std::tuple<bool, int> get_flip_value(bool flip_horizontal, bool flip_vertical) {
    bool flip_image = true;
    int flip_value;
    if (flip_horizontal && flip_vertical) {
        flip_value = -1;  // flip both, horizontal and vertical
    }
    else if (flip_horizontal) {
        flip_value = 1;
    }
    else if (flip_vertical) {
        flip_value = 0;
    }
    else {
        flip_image = false;
    }
    return std::make_tuple(flip_image, flip_value);
}

void consume_frames(const ros::NodeHandle &nh,
                    ImageCapture &cap,
                    double fps = 240.0,
                    std::string topic = "camera",
                    const std::string frame_id = "camera",
                    const std::string camera_info_url = "",
                    const bool flip_image = false,
                    const int flip_value = 0) {
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("camera", 1);

    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;

    std_msgs::Header header;
    header.frame_id = frame_id;
    camera_info_manager::CameraInfoManager cam_info_manager(nh, topic, camera_info_url);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;

    ros::Rate r(fps);
    cv::Mat frame;
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();

        if (cap.empty()) {
            continue;
        }

        cap.pull(frame);
        // Check if grabbed frame is actually filled with some content
        // If so, is anyone listening?
        if (frame.empty() ||
            pub.getNumSubscribers() == 0) {
            continue;
        }

        // Flip the image if necessary
        if (flip_image) {
            cv::flip(frame, frame, flip_value);
        }
        msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        // Create a default camera info if we didn't get a stored one on initialization
        if (cam_info_msg.distortion_model == "") {
            ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
            cam_info_msg = get_default_camera_info_from_image_msg(
                    boost::const_pointer_cast<const sensor_msgs::Image>(msg));
            cam_info_manager.setCameraInfo(cam_info_msg);
        }

        // The timestamps are in sync thanks to this publisher
        pub.publish(*msg, cam_info_msg, ros::Time::now());
    }
}
void consume_frames(ImageCapture &cap,
                    double fps = 240.0,
                    std::string topic = "camera",
                    const std::string frame_id = "camera",
                    const std::string camera_info_url = "",
                    const bool flip_image = false,
                    const int flip_value = 0) {
    ros::NodeHandle nh(ros::this_node::getName());
    consume_frames(nh, cap, fps, topic, frame_id, camera_info_url, flip_image, flip_value);
}
}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_OPENCV_UTILITY_HPP_
