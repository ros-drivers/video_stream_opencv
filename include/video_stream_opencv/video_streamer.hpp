#ifndef _VIDEO_STREAM_OPENCV_ROS_VIDEO_STREAMER_HPP_
#define _VIDEO_STREAM_OPENCV_ROS_VIDEO_STREAMER_HPP_

#include <functional>
#include <thread>
#include <tuple>

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <opencv2/videoio.hpp>

#include <video_stream_opencv/image_capture.hpp>
#include <video_stream_opencv/utility.hpp>

#define GET_PARAM_PTR(NH, T, VAR, DEFAULT, MESSAGE)                           \
    T VAR;                                                                    \
    NH->param(#VAR, VAR, T(DEFAULT));                                         \
    ROS_INFO_STREAM(MESSAGE << VAR);

namespace video_stream_opencv {
struct VideoStreamer {
    VideoStreamer(): VideoStreamer(boost::make_shared<ros::NodeHandle>("~"))
    {}
    VideoStreamer(ros::NodeHandlePtr nh_ptr, std::string video_stream_provider = ""):
        nh_ptr_(nh_ptr) {
        update_srv_ = nh_ptr_->advertiseService("update_param",
                                                &VideoStreamer::update_cb_,
                                                this);
        update_parameters_(video_stream_provider);
    }

    bool update_cb_(std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response &res) {
        return update_parameters_();
    }

    bool update_parameters_(std::string video_stream_provider="") {
        if (!video_stream_provider.size()) {
            nh_ptr_->getParam("video_stream_provider", video_stream_provider);
        }
        if (video_stream_provider.size()) {
            ROS_INFO_STREAM("Resource video_stream_provider: " << video_stream_provider);
        }
        else {
            ROS_ERROR("Failed to get param 'video_stream_provider'");
            return false;
        }

        GET_PARAM_PTR(nh_ptr_, std::string, camera_name, "camera", "Camera name: ");
        camera_name_ = std::move(camera_name);

        GET_PARAM_PTR(nh_ptr_, std::string, frame_id, camera_name_, "Frame id in image header: ");
        frame_id_ = std::move(frame_id);

        GET_PARAM_PTR(nh_ptr_, std::string, camera_info_url, "", "camera_info_url: ");
        camera_info_url_ = std::move(camera_info_url);

        GET_PARAM_PTR(nh_ptr_, double, camera_fps, 30.0, "Query rate for images from camera: ");
        if (camera_fps <= 0) {
            ROS_ERROR_STREAM("Camera FPS can't be 0 or negative, setting to default value");
            camera_fps = 30.0;
        }

        GET_PARAM_PTR(nh_ptr_, bool, flip_horizontal, false, "Flip horizontally? : ");

        GET_PARAM_PTR(nh_ptr_, bool, flip_vertical, false, "Flip vertically? : ");

        GET_PARAM_PTR(nh_ptr_, bool, loop, true, "Reopen stream on failure: ");

        GET_PARAM_PTR(nh_ptr_, int, max_error, 0, "Maximum allowed errors (0 for infinity): ");
        if (max_error < 0) {
            ROS_ERROR_STREAM("Error threshold can't be negative, setting to default value");
            max_error = 0;
        }
        max_error_ = max_error;

        GET_PARAM_PTR(nh_ptr_, int, buffer_queue_size, 100, "Buffer size for captured frames: ");
        if (buffer_queue_size < 0) {
            ROS_ERROR_STREAM("Buffer size can't be negative, setting to default value");
            buffer_queue_size = 100;
        }

        GET_PARAM_PTR(nh_ptr_, double, fps, 240.0, "Throttling rate for image publisher: ");
        fps_ = fps;
        if (fps_ <= 0) {
            ROS_ERROR_STREAM("Publishing rate can't be 0 or negative, setting to default value");
            fps_ = 240.0;
        }

        if (video_stream_provider.size() < 4 && fps_ > camera_fps) {
            ROS_WARN_STREAM("Asked to publish at 'fps' (" << fps_ <<
                            ") which is higher than the 'camera_fps' (" << camera_fps <<
                            "), we can't publish faster than the camera provides images.");
        }

        int width_target, height_target;
        nh_ptr_->param("width", width_target, 0);
        nh_ptr_->param("height", height_target, 0);
        if (width_target && height_target) {
            ROS_INFO_STREAM("Forced image width is: " << width_target);
            ROS_INFO_STREAM("Forced image height is: " << height_target);
        }

        std::tie(flip_image_, flip_value_) = get_flip_value(flip_horizontal, flip_vertical);

        if (img_cap_ && img_cap_->get_video_stream_provider() == video_stream_provider) {
            img_cap_->close();
        }

        auto cap = std::make_unique<ImageCapture>(video_stream_provider,
                                                  camera_fps,
                                                  static_cast<unsigned int>(buffer_queue_size),
                                                  loop);
        if(!cap->isOpened()) {
            ROS_ERROR_STREAM("Could not open new stream.");
            return false;
        }

        cap->set(cv::CAP_PROP_FPS, camera_fps);
        if (width_target > 0 && height_target > 0) {
            cap->set(cv::CAP_PROP_FRAME_WIDTH, width_target);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_target);
        }

        // OpenCV 2.4 returns -1 (instead of a 0 as the spec says) and prompts an error
        // HIGHGUI ERROR: V4L2: Unable to get property <unknown property string>(5) - Invalid argument
        double reported_camera_fps = cap->get(cv::CAP_PROP_FPS);
        if (reported_camera_fps > 0.0) {
            ROS_INFO_STREAM("Camera reports FPS: " << reported_camera_fps);
        }
        else {
            ROS_INFO_STREAM("Backend can't provide camera FPS information");
        }

        img_cap_.reset(cap.release());
        return true;
    }

    int run() {
        if(!img_cap_->isOpened()) {
            ROS_ERROR_STREAM("Could not open the stream.");
            return -1;
        }
        ROS_INFO_STREAM("Opened the stream, starting to publish.");
        std::thread cap_thread{[&]() { return capture_frames(img_cap_,
                static_cast<unsigned int>(max_error_)); }};

        consume_frames(*nh_ptr_, img_cap_, fps_, camera_name_, frame_id_, camera_info_url_,
                       flip_image_, flip_value_);
        cap_thread.join();
        return 0;
    }

  private:
    ros::NodeHandlePtr nh_ptr_;
    std::unique_ptr<ImageCapture> img_cap_;
    ros::ServiceServer update_srv_;

    unsigned int max_error_;
    double fps_;
    std::string camera_name_, frame_id_, camera_info_url_;
    bool flip_image_;
    int flip_value_;
};  // class VideoStreamer
}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_OPENCV_ROS_VIDEO_STREAMER_HPP_
