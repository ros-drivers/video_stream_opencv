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
  *     * loop: bool, defaults to true, reopens video_stream_provider if true
  *     * max_error: unsigned int, defaults to 0, max number of continuous
  *                  errors tolerated before quitting. 0 is 
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_stream");
    VideoStreamer node;
    return node.run();
}
