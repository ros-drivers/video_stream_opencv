/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Sammy Pfeiffer
 */

#include <cstdint>
#include <functional>
#include <thread>

#include <boost/pointer_cast.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <video_stream_opencv/image_capture.hpp>

using namespace video_stream_opencv;

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
        cap >> frame;
        if (cap.isFile()) {
            camera_fps_rate.sleep();
        }
        if(!frame.empty()) {
            cap.push(frame.clone());
        }
    }
}

/*
 * If flip_image:bool is false, using flip_value:int is undefined behavior
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

    // Create a default camera info if we didn't get a stored one on initialization
    if (cam_info_msg.distortion_model == "") {
        ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
        cam_info_msg = get_default_camera_info_from_image_msg(
                boost::const_pointer_cast<const sensor_msgs::Image>(msg));
        cam_info_manager.setCameraInfo(cam_info_msg);
    }

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
    ros::NodeHandle nh;
    consume_frames(nh, cap, fps, topic, frame_id, camera_info_url, flip_image, flip_value);
}

#define GET_PARAM_NAME(NH, T, VAR, NAME, DEFAULT, MESSAGE)                    \
    T VAR;                                                                    \
    NH.param(NAME, VAR, T(DEFAULT));                                          \
    ROS_INFO_STREAM(MESSAGE << VAR);
#define GET_PARAM(NH, T, VAR, DEFAULT, MESSAGE)                               \
    T VAR;                                                                    \
    NH.param(#VAR, VAR, T(DEFAULT));                                          \
    ROS_INFO_STREAM(MESSAGE << VAR);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle _nh("~"); // to get the private params

    // provider can be an url (e.g.: rtsp://10.0.0.1:554) or a number of device, (e.g.: 0 would be /dev/video0)
    std::string video_stream_provider;
    if (_nh.getParam("video_stream_provider", video_stream_provider)) {
        ROS_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider);
    }
    else {
        ROS_ERROR("Failed to get param 'video_stream_provider'");
        return -1;
    }

    GET_PARAM(_nh, std::string, camera_name, "camera", "Camera name: ");

    GET_PARAM(_nh, double, camera_fps, 30.0, "Query rate for images from camera: ");

    GET_PARAM(_nh, int, buffer_queue_size, 100, "Buffer size for captured frames: ");
    if (buffer_queue_size < 0) {
        ROS_ERROR_STREAM("Buffer size can't be negative, setting to default value");
        buffer_queue_size = 100;
    }

    GET_PARAM(_nh, double, fps, 240.0, "Throttling rate for image publisher: ");

    if (video_stream_provider.size() < 4 && fps > camera_fps) {
        ROS_WARN_STREAM("Asked to publish at 'fps' (" << fps <<
                        ") which is higher than the 'camera_fps' (" << camera_fps <<
                        "), we can't publish faster than the camera provides images.");
    }

    GET_PARAM(_nh, std::string, frame_id, camera_name, "Frame id in image header: ");

    GET_PARAM(_nh, std::string, camera_info_url, "", "camera_info_url: ");

    GET_PARAM(_nh, bool, flip_horizontal, false, "Flip horizontally? : ");

    GET_PARAM(_nh, bool, flip_vertical, false, "Flip vertically? : ");

    int width_target, height_target;
    _nh.param("width", width_target, 0);
    _nh.param("height", height_target, 0);
    if (width_target && height_target) {
        ROS_INFO_STREAM("Forced image width is: " << width_target);
        ROS_INFO_STREAM("Forced image height is: " << height_target);
    }

    bool flip_image;
    int flip_value;
    std::tie(flip_image, flip_value) = get_flip_value(flip_horizontal, flip_vertical);

    ImageCapture cap{video_stream_provider, camera_fps, uint16_t(buffer_queue_size), true};
    if(!cap.isOpened()) {
        ROS_ERROR_STREAM("Could not open the stream.");
        return -1;
    }

    cap.set(CV_CAP_PROP_FPS, camera_fps);
    if (width_target && height_target) {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, width_target);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, height_target);
    }

    // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
    // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
    double reported_camera_fps = cap.get(CV_CAP_PROP_FPS);
    if (reported_camera_fps > 0.0) {
        ROS_INFO_STREAM("Camera reports FPS: " << reported_camera_fps);
    }
    else {
        ROS_INFO_STREAM("Backend can't provide camera FPS information");
    }

    ROS_INFO_STREAM("Opened the stream, starting to publish.");
    std::thread cap_thread{capture_frames, std::ref(cap)};
    consume_frames(cap, fps, camera_name, frame_id, camera_info_url, flip_image, flip_value);
    cap_thread.join();
}
