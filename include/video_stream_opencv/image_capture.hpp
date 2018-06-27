#ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
#define _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_

#include <algorithm>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace video_stream_opencv {
namespace {
const double _CAPTURE_FPS = 30;
const unsigned int _MAX_QUEUE_SIZE = 100;
const bool _DEFAULT_LOOP = false;
}

struct ImageCapture
{
    const double camera_fps = _CAPTURE_FPS;

    ImageCapture(const std::string stream_path, double fps,
                 unsigned int queue_size, bool loop_file = _DEFAULT_LOOP):
        camera_fps(fps),
        max_queue_size_(queue_size),
        loop_(loop_file),
        stream_path_(stream_path) {
        video_stream_provider_ = StreamType::UNKNOWN;
        // same size limit used internally in cv::VideoCapture
        if (stream_path.size() < 4) {
            video_stream_provider_ = StreamType::DEV_VIDEO;
        }
        else if (stream_path.rfind("http://", 0) == 0 ||
                 stream_path.rfind("https://", 0) == 0) {
            video_stream_provider_ = StreamType::HTTP;
        }
        else if (stream_path.rfind("rtsp://", 0) == 0) {
            video_stream_provider_ = StreamType::RTSP;
        }
        // one single % is used to identify img_sequence patterns by cv
        else if (std::count(stream_path.begin(), stream_path.end(), '%') == 1) {
            video_stream_provider_ = StreamType::IMG_SEQ;
        }
        else {
            std::ifstream ifs(stream_path);
            // Check if file exists to know if it's a valid file
            if (ifs.good()) {
                // can be a video or image file, both are valid
                video_stream_provider_ = StreamType::MEDIA_FILE;
            }
        }
        open_();
    }

    ImageCapture(const std::string &stream_path, double fps = _CAPTURE_FPS):
        ImageCapture(stream_path, fps, _MAX_QUEUE_SIZE, _DEFAULT_LOOP)
    {}

    ImageCapture(const std::string &stream_path, unsigned int queue_size):
        ImageCapture(stream_path, _CAPTURE_FPS, queue_size, _DEFAULT_LOOP)
    {}

    ImageCapture(const std::string &stream_path, bool loop):
        ImageCapture(stream_path, _CAPTURE_FPS, _MAX_QUEUE_SIZE, loop)
    {}

    ImageCapture(const std::string &stream_path, double fps, bool loop):
        ImageCapture(stream_path, fps, _MAX_QUEUE_SIZE, loop)
    {}

    bool isFile() const {
        return (video_stream_provider_ == StreamType::MEDIA_FILE ||
                video_stream_provider_ == StreamType::IMG_SEQ);
    }

    // wrap get, set, isOpened, grab, retrieve, read and >> for VideoCapture
    double get(int propId) const {
        return cap_.get(propId);
    }

    bool set(int propId, double value) {
        return cap_.set(propId, value);
    }

    bool isOpened() const {
        return cap_.isOpened();
    }

    bool grab() {
        bool success = cap_.grab();
        if (success || !loop_) {
            return success;
        }
        // nothing has changed wrt StreamType
        open_();
        return grab();
    }

    bool retrieve(cv::OutputArray &arr) const {
        return cap_.retrieve(arr);
    }

    bool read(cv::OutputArray &arr) {
        if (grab()) {
            return retrieve(arr);
        }
        else {
            arr.release();
        }
        return arr.empty();
    }

    ImageCapture& operator>> (cv::UMat &img) {
        read(img);
        return *this;
    }

    ImageCapture& operator>> (cv::Mat &img) {
        read(img);
        return *this;
    }

    // wrap empty, pop, push for frames_queue_
    bool empty() const {
        std::lock_guard<std::mutex> lock(q_mutex_);
        return frames_queue_.empty();
    }

    void pop(cv::Mat &img) {
        std::lock_guard<std::mutex> lock(q_mutex_);
        img = frames_queue_.front();
        frames_queue_.pop();
    }

    bool push(const cv::Mat &img) {
        std::lock_guard<std::mutex> lock(q_mutex_);
        bool full = false;
        if (frames_queue_.size() >= max_queue_size_) {
            full = true;
            frames_queue_.pop();
        }
        frames_queue_.push(img);
        return full;
    }

    void try_pop(cv::Mat &img) {
        std::lock_guard<std::mutex> lock(q_mutex_);
        if (frames_queue_.empty()) {
            img.release();
            return;
        }
        img = frames_queue_.front();
        frames_queue_.pop();
    }

    private:
        mutable std::mutex q_mutex_;
        mutable std::queue<cv::Mat> frames_queue_;
        mutable cv::VideoCapture cap_;

        const unsigned int max_queue_size_ = _MAX_QUEUE_SIZE;
        const bool loop_ = _DEFAULT_LOOP;
        const std::string stream_path_;

        enum class StreamType { UNKNOWN, DEV_VIDEO, HTTP, RTSP, MEDIA_FILE, IMG_SEQ };
        StreamType video_stream_provider_;

        bool open_() {
            if (video_stream_provider_ == StreamType::DEV_VIDEO) {
                return cap_.open(atoi(stream_path_.c_str()));
            }
            return cap_.open(stream_path_);
        }
};
}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
