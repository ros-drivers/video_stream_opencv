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
#include <tuple>

#include <ros/ros.h>

#include <opencv2/videoio.hpp>

#include <video_stream_opencv/video_stream_opencv.hpp>

using namespace video_stream_opencv;

/**
  * Compulsory parameter:
  *     * video_stream_provider: string, same as input to cv::VideoCapture
  *
  * Optional parameters:
  *     * camera_name: string, defaults to "camera"
  *     * frame_id: string, defaults to camera_name
  *     * camera_info_url: string, used by camera_info_manager to retrieve
  *                        camera settings
  *     * camera_fps: double, defaults to 30
  *     * flip_horizontal: bool, defaults to false, image is flipped if true
  *     * flip_vertical: bool, defaults to false, image is flipped if true
  *     * buffer_queue_size: unsigned int, defaults to 100, used to maintain
  *                          a buffer for streaming data
  *     * fps: double, defaults to 240, throttles the output
  *     * width: int, defaults to 0
  *     * height: int, defaults to 0
  *     If both width and height are more than 0, VideoCapture is used to
  *     modify the image at input itself
  *
  * Set the parameters from command line by
  * $ rosrun video_stream_opencv video_stream _parameter_name:=value
  *
  * For values of type string, ensure that the values aren't numerical.
  * Eg: for video_stream_provider, instead of 0, use /dev/video0
  */

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

    GET_PARAM(_nh, std::string, frame_id, camera_name, "Frame id in image header: ");

    GET_PARAM(_nh, std::string, camera_info_url, "", "camera_info_url: ");

    GET_PARAM(_nh, double, camera_fps, 30.0, "Query rate for images from camera: ");

    GET_PARAM(_nh, bool, flip_horizontal, false, "Flip horizontally? : ");

    GET_PARAM(_nh, bool, flip_vertical, false, "Flip vertically? : ");

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

    cap.set(cv::CAP_PROP_FPS, camera_fps);
    if (width_target > 0 && height_target > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width_target);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_target);
    }

    // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
    // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
    double reported_camera_fps = cap.get(cv::CAP_PROP_FPS);
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
    return 0;
}
