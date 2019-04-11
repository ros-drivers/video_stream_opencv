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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>
#include <queue>
#include <mutex>


namespace video_stream_opencv {

class VideoStreamNodelet: public nodelet::Nodelet {
protected:
boost::shared_ptr<ros::NodeHandle> nh, pnh;
image_transport::CameraPublisher pub;
std::mutex q_mutex;
std::queue<cv::Mat> framesQueue;
cv::Mat frame;
boost::shared_ptr<cv::VideoCapture> cap;
std::string video_stream_provider;
std::string video_stream_provider_type;
std::string frame_id;
double set_camera_fps;
double fps;
int max_queue_size;
bool loop_videofile;
bool is_subscribed;
int width_target;
int height_target;
bool flip_image;
int flip_value;
boost::thread capture_thread;
ros::Timer publish_timer;
sensor_msgs::CameraInfo cam_info_msg;

// Based on the ros tutorial on transforming opencv images to Image messages

virtual sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr img){
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    NODELET_INFO_STREAM("The image width is: " << img->width);
    NODELET_INFO_STREAM("The image height is: " << img->height);
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
            (0.0) (1.0) (img->height/2.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
            (0.0) (1.0) (0.0)
            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
            (0.0) (1.0) (img->height/2.0) (0.0)
            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}


virtual void do_capture() {
    NODELET_DEBUG("Capture thread started");
    cv::Mat frame;
    ros::Rate camera_fps_rate(set_camera_fps);

    int frame_counter = 0;
    // Read frames as fast as possible
    while (nh->ok() && is_subscribed) {
        if (!cap->read(frame))
        {
          NODELET_FATAL("Could not capture frame!");
          continue;
        }

        frame_counter++;
        if (video_stream_provider_type == "videofile")
        {
            camera_fps_rate.sleep();
        }
        if (video_stream_provider_type == "videofile" &&
            frame_counter == cap->get(CV_CAP_PROP_FRAME_COUNT)) 
        {
            if (loop_videofile)
            {
                cap->open(video_stream_provider);
                frame_counter = 0;
            }
            else {
              NODELET_INFO("Reached the end of frames");
              break;
            }
        }

        if(!frame.empty()) {
            std::lock_guard<std::mutex> g(q_mutex);
            // accumulate only until max_queue_size
            if (framesQueue.size() < max_queue_size) {
                framesQueue.push(frame.clone());
            }
            // once reached, drop the oldest frame
            else {
                framesQueue.pop();
                framesQueue.push(frame.clone());
            }
        }
    }
    NODELET_DEBUG("Capture thread finished");
}

virtual void do_publish(const ros::TimerEvent& event) {
    bool is_new_image = false;
    sensor_msgs::ImagePtr msg;
    std_msgs::Header header;
    header.frame_id = frame_id;
    {
        std::lock_guard<std::mutex> g(q_mutex);
        if (!framesQueue.empty()){
            frame = framesQueue.front();
            framesQueue.pop();
            is_new_image = true;
        }
    }

    // Check if grabbed frame is actually filled with some content
    if(!frame.empty()) {
        // Flip the image if necessary
        if (flip_image && is_new_image)
            cv::flip(frame, frame, flip_value);
        msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        // Create a default camera info if we didn't get a stored one on initialization
        if (cam_info_msg.distortion_model == ""){
            NODELET_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
            cam_info_msg = get_default_camera_info_from_image(msg);
            // cam_info_manager.setCameraInfo(cam_info_msg);
        }
        // The timestamps are in sync thanks to this publisher
        pub.publish(*msg, cam_info_msg, ros::Time::now());
    }
}

virtual void subscribe() {
  ROS_DEBUG("Subscribe");
  cap.reset(new cv::VideoCapture);
  if (video_stream_provider_type == "videodevice") {
    NODELET_INFO_STREAM("Opening VideoCapture with provider: /dev/video" << video_stream_provider);
    cap->open(std::stoi(video_stream_provider));
  } else {
    NODELET_INFO_STREAM("Opening VideoCapture with provider: " << video_stream_provider);
    cap->open(video_stream_provider);
  }

  double reported_camera_fps;
  // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
  // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
  reported_camera_fps = cap->get(CV_CAP_PROP_FPS);
  if (reported_camera_fps > 0.0)
    NODELET_INFO_STREAM("Camera reports FPS: " << reported_camera_fps);
  else
    NODELET_INFO_STREAM("Backend can't provide camera FPS information");

  cap->set(CV_CAP_PROP_FPS, set_camera_fps);
  if(!cap->isOpened()){
    NODELET_ERROR_STREAM("Could not open the stream.");
    return;
  }
  if (width_target != 0 && height_target != 0){
    cap->set(CV_CAP_PROP_FRAME_WIDTH, width_target);
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, height_target);
  }
  is_subscribed = true;
  try {
    capture_thread = boost::thread(
      boost::bind(&VideoStreamNodelet::do_capture, this));
    publish_timer = nh->createTimer(
      ros::Duration(1.0 / fps), &VideoStreamNodelet::do_publish, this);
  } catch (std::exception& e) {
    NODELET_ERROR_STREAM(e.what());
    is_subscribed = false;
  }
}

virtual void unsubscribe() {
  ROS_DEBUG("Unsubscribe");
  publish_timer.stop();
  is_subscribed = false;
  capture_thread.join();
  cap.reset();
}

virtual void connectionCallbackImpl() {
  bool always_subscribe = false;
  pnh->getParamCached("always_subscribe", always_subscribe);
  if ((video_stream_provider == "videofile" || always_subscribe) && !is_subscribed) {
    subscribe();
  } else {
    if (pub.getNumSubscribers() > 0) {
    if (!is_subscribed) subscribe();
    } else {
      if (is_subscribed) unsubscribe();
    }
  }
}

virtual void connectionCallback(const image_transport::SingleSubscriberPublisher&) {
  connectionCallbackImpl();
}

virtual void infoConnectionCallback(const ros::SingleSubscriberPublisher&) {
  connectionCallbackImpl();
}


virtual void onInit() {
    bool use_multithread;
    ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
    if (use_multithread)
    {
      NODELET_DEBUG("Using multithread callback");
      nh.reset (new ros::NodeHandle(getMTNodeHandle()));
      pnh.reset (new ros::NodeHandle(getMTPrivateNodeHandle()));
    }
    else
    {
      NODELET_DEBUG("Using singlethread callback");
      nh.reset(new ros::NodeHandle(getNodeHandle()));
      pnh.reset(new ros::NodeHandle(getPrivateNodeHandle()));
    }

    // provider can be an url (e.g.: rtsp://10.0.0.1:554) or a number of device, (e.g.: 0 would be /dev/video0)
    if (pnh->getParam("video_stream_provider", video_stream_provider)){
        NODELET_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider);
        // If we are given a string of 4 chars or less (I don't think we'll have more than 100 video devices connected)
        // treat is as a number and act accordingly so we open up the videoNUMBER device
        if (video_stream_provider.size() < 4){
            NODELET_INFO_STREAM("Getting video from provider: /dev/video" << video_stream_provider);
            video_stream_provider_type = "videodevice";
        }
        else{
            NODELET_INFO_STREAM("Getting video from provider: " << video_stream_provider);
            if (video_stream_provider.find("http://") != std::string::npos ||
                    video_stream_provider.find("https://") != std::string::npos){
                video_stream_provider_type = "http_stream";
            }
            else if(video_stream_provider.find("rtsp://") != std::string::npos){
                video_stream_provider_type = "rtsp_stream";
            }
            else {
                // Check if file exists to know if it's a videofile
                std::ifstream ifs;
                ifs.open(video_stream_provider.c_str(), std::ifstream::in);
                if (ifs.good()){
                    video_stream_provider_type = "videofile";
                }
                else
                    video_stream_provider_type = "unknown";
            }
        }
    }
    else{
        NODELET_ERROR("Failed to get param 'video_stream_provider'");
        return;
    }

    NODELET_INFO_STREAM("Video stream provider type detected: " << video_stream_provider_type);

    std::string camera_name;
    pnh->param("camera_name", camera_name, std::string("camera"));
    NODELET_INFO_STREAM("Camera name: " << camera_name);

    pnh->param("set_camera_fps", set_camera_fps, 30.0);
    NODELET_INFO_STREAM("Setting camera FPS to: " << set_camera_fps);

    int buffer_queue_size;
    pnh->param("buffer_queue_size", buffer_queue_size, 100);
    NODELET_INFO_STREAM("Setting buffer size for capturing frames to: " << buffer_queue_size);
    max_queue_size = buffer_queue_size;

    pnh->param("fps", fps, 240.0);
    NODELET_INFO_STREAM("Throttling to fps: " << fps);

    if (video_stream_provider.size() < 4 && fps > set_camera_fps)
        NODELET_WARN_STREAM("Asked to publish at 'fps' (" << fps
                        << ") which is higher than the 'set_camera_fps' (" << set_camera_fps <<
                        "), we can't publish faster than the camera provides images.");

    pnh->param("frame_id", frame_id, std::string("camera"));
    NODELET_INFO_STREAM("Publishing with frame_id: " << frame_id);

    std::string camera_info_url;
    pnh->param("camera_info_url", camera_info_url, std::string(""));
    NODELET_INFO_STREAM("Provided camera_info_url: '" << camera_info_url << "'");

    bool flip_horizontal;
    pnh->param("flip_horizontal", flip_horizontal, false);
    NODELET_INFO_STREAM("Flip horizontal image is: " << ((flip_horizontal)?"true":"false"));

    bool flip_vertical;
    pnh->param("flip_vertical", flip_vertical, false);
    NODELET_INFO_STREAM("Flip vertical image is: " << ((flip_vertical)?"true":"false"));

    pnh->param("width", width_target, 0);
    pnh->param("height", height_target, 0);
    if (width_target != 0 && height_target != 0){
        NODELET_INFO_STREAM("Forced image width is: " << width_target);
        NODELET_INFO_STREAM("Forced image height is: " << height_target);
    }

    // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
    // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
    flip_image = true;
    if (flip_horizontal && flip_vertical)
        flip_value = -1; // flip both, horizontal and vertical
    else if (flip_horizontal)
        flip_value = 1;
    else if (flip_vertical)
        flip_value = 0;
    else
        flip_image = false;

    pnh->param("loop_videofile", loop_videofile, false);

    std_msgs::Header header;
    header.frame_id = frame_id;
    camera_info_manager::CameraInfoManager cam_info_manager(*nh, camera_name, camera_info_url);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;

    image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&VideoStreamNodelet::connectionCallback, this, _1);
    ros::SubscriberStatusCallback info_connect_cb =
      boost::bind(&VideoStreamNodelet::infoConnectionCallback, this, _1);
    pub = image_transport::ImageTransport(*nh).advertiseCamera(
      "image_raw", 1,
      connect_cb, connect_cb,
      info_connect_cb, info_connect_cb,
      ros::VoidPtr(), false);
}
};
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_stream_opencv::VideoStreamNodelet, nodelet::Nodelet)
