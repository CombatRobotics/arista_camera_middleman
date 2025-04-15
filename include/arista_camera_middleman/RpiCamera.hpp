#ifndef RPI_CAMERA_HPP
#define RPI_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>

class FrameReader {
public:
    explicit FrameReader(size_t size) : buffer(size), offset(0), size(size) {}

    void pushQueue(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        offset = (offset + 1) % size;
        buffer[offset] = frame.clone();
    }

    cv::Mat popQueue() {
        std::lock_guard<std::mutex> lock(mutex);
        int index = (offset == 0) ? size - 1 : offset - 1;
        return buffer[index].clone();
    }

private:
    std::vector<cv::Mat> buffer;
    int offset;
    size_t size;
    std::mutex mutex;
};

class Camera : public rclcpp::Node {
public:
    Camera(std::string topic_name = "/camera/image_raw", int frame_queue_size = 5)
        : Node("rpi_camera_reader"), frame(frame_queue_size), is_running(false), window_name("Arducam PTZ Camera Controller Preview")
    {
        using std::placeholders::_1;
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name, 10,
            std::bind(&Camera::imageCallback, this, _1)
        );
    }

    void start_preview(int wait_ms = 1) {
        is_running = true;
        preview_thread = std::thread(&Camera::previewLoop, this, wait_ms);
    }

    void stop_preview() {
        is_running = false;
        if (preview_thread.joinable()) {
            preview_thread.join();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            frame.pushQueue(img);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    cv::Mat getFrame() {
        return frame.popQueue();
    }
    void blocking_preview() {
        while (rclcpp::ok()) {
            cv::Mat frame_mat = getFrame();
            if (!frame_mat.empty()) {
                cv::imshow(window_name, frame_mat);
                if (cv::waitKey(1) == 'q') break;
            }
        }
        cv::destroyWindow(window_name);
    }
    private:

    void previewLoop(int wait_ms) {
        bool window_created = false;

        while (is_running && rclcpp::ok()) {
            cv::Mat frame_mat = frame.popQueue();
            if (!frame_mat.empty()) {
                cv::imshow(window_name, frame_mat);
                window_created = true;
                if (cv::waitKey(wait_ms) == 'q') {
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        }

        if (window_created) {
            cv::destroyWindow(window_name);
        }
    }

    FrameReader frame;
    std::atomic<bool> is_running;
    std::string window_name;
    std::thread preview_thread;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};

#endif // RPI_CAMERA_HPP
