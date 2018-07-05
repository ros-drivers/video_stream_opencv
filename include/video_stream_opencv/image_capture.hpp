#ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
#define _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_

#include <algorithm>
#include <fstream>
#include <mutex>
#include <queue>
#include <shared_mutex>
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
  private:
    mutable std::mutex q_mutex_;
    mutable std::recursive_mutex cap_mutex_;
    mutable std::queue<cv::Mat> frames_queue_;
    mutable cv::VideoCapture cap_;

    unsigned int max_queue_size_;
    bool loop_;
    std::string stream_path_;

    enum class StreamType { UNKNOWN, DEV_VIDEO, HTTP, RTSP, MEDIA_FILE, IMG_SEQ };
    StreamType video_stream_provider_;

    bool open_() {
        if (video_stream_provider_ == StreamType::DEV_VIDEO) {
            return cap_.open(atoi(stream_path_.c_str()));
        }
        return cap_.open(stream_path_);
    }
  public:
    double camera_fps;

    ImageCapture(std::string stream_path,
                 double fps = _CAPTURE_FPS,
                 unsigned int queue_size = _MAX_QUEUE_SIZE,
                 bool loop_file = _DEFAULT_LOOP) {
            reset(stream_path, fps, queue_size, loop_file);
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

    ~ImageCapture() {
        dtor_mutex.lock();
        q_mutex_.lock();
        cap_mutex_.lock();
    }

    void reset(std::string stream_path,
               double fps = _CAPTURE_FPS,
               unsigned int queue_size = _MAX_QUEUE_SIZE,
               bool loop_file = _DEFAULT_LOOP) {
        std::lock_guard q_lock(q_mutex_);
        std::lock_guard cap_lock(cap_mutex_);
        camera_fps = fps;
        max_queue_size_ = queue_size;
        loop_ = loop_file;
        stream_path_ = stream_path;
        video_stream_provider_ = StreamType::UNKNOWN;
        frames_queue_ = {};

        // hack to remove spurious quotation marks
        if (stream_path.size() > 2) {
            switch (stream_path[0]) {
                case '\'':
                    [[fallthrough]];
                case '"':
                        if (stream_path.front() == stream_path.back()) {
                            // remove the front and back
                            stream_path = stream_path.substr(1, stream_path.size() -2);
                        }
            }
        }

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

    void reset() {
        // Why though?
        return reset(stream_path_, camera_fps, max_queue_size_, loop_);
    }

    void close() {
        std::lock_guard cap_lock(cap_mutex_);
        cap_.release();
    }

    bool isFile() const {
        std::lock_guard cap_lock(cap_mutex_);
        return (video_stream_provider_ == StreamType::MEDIA_FILE ||
                video_stream_provider_ == StreamType::IMG_SEQ);
    }

    // wrap get, set, isOpened, grab, retrieve, read and >> for VideoCapture
    double get(int propId) const {
        std::lock_guard cap_lock(cap_mutex_);
        return cap_.get(propId);
    }

    bool set(int propId, double value) {
        std::lock_guard cap_lock(cap_mutex_);
        return cap_.set(propId, value);
    }

    bool isOpened() const {
        std::lock_guard cap_lock(cap_mutex_);
        return cap_.isOpened();
    }

    bool grab() {
        std::lock_guard cap_lock(cap_mutex_);
        bool success = cap_.grab();
        if (success || !loop_) {
            return success;
        }
        // nothing has changed wrt StreamType
        open_();
        return grab();
    }

    bool retrieve(cv::OutputArray &arr) const {
        std::lock_guard cap_lock(cap_mutex_);
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

    // wrap clear, empty, pop, push for frames_queue_
    void clear() {
        std::lock_guard q_lock(q_mutex_);
        frames_queue_ = {};
    }

    bool empty() const {
        std::lock_guard q_lock(q_mutex_);
        return frames_queue_.empty();
    }

    void pop(cv::Mat &img) {
        std::lock_guard q_lock(q_mutex_);
        img = frames_queue_.front();
        frames_queue_.pop();
    }

    bool push(const cv::Mat &img) {
        std::lock_guard q_lock(q_mutex_);
        bool full = false;
        if (frames_queue_.size() >= max_queue_size_) {
            full = true;
            frames_queue_.pop();
        }
        frames_queue_.push(img);
        return full;
    }

    void try_pop(cv::Mat &img) {
        std::lock_guard q_lock(q_mutex_);
        if (frames_queue_.empty()) {
            img.release();
            return;
        }
        img = frames_queue_.front();
        frames_queue_.pop();
    }

    std::string get_video_stream_provider() const {
        return stream_path_;
    }

    mutable std::shared_mutex dtor_mutex;
};
}  // namespace video_stream_opencv

#endif  // ifndef _VIDEO_STREAM_CAPTURE_IMAGE_CAPTURE_HPP_
