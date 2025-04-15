#ifndef AUTOFOCUS_HPP
#define AUTOFOCUS_HPP

#include "arista_camera_middleman/CamFocuser.hpp"
#include "arista_camera_middleman/RpiCamera.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <tuple>
#include <algorithm> 

class AutoFocus {
public:
    AutoFocus(Focuser* focuser, Camera* camera)
        : focuser(focuser), camera(camera), debug(false), MAX_FOCUS_VALUE(18000) {}

    void setDebug(bool enable) { debug = enable; }

    int getEndPoint() {
        int zoom = focuser->get(Focuser::OPT_ZOOM) / 1000;
        int index = std::clamp(zoom, 0, static_cast<int>(focuser->end_point.size()) - 1);
        int val = focuser->end_point[index];
        if (debug) std::cout << "End Point: " << val << std::endl;
        return val;
    }

    int getStartingPoint() {
        int zoom = std::ceil(focuser->get(Focuser::OPT_ZOOM) / 1000.0);
        int index = std::clamp(zoom, 0, static_cast<int>(focuser->starting_point.size()) - 1);
        int val = focuser->starting_point[index];
        if (debug) std::cout << "Starting Point: " << val << std::endl;
        return val;
    }

    double laplacianVariance(const cv::Mat& img) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::Mat lap;
        cv::Laplacian(gray, lap, CV_64F);
        return cv::mean(lap.mul(lap))[0];
    }

    double filteredValue(double value) {
        const size_t max_len = 3;
        value_buffer.push_back(value);
        if (value_buffer.size() > max_len) value_buffer.erase(value_buffer.begin());
        std::vector<double> sorted = value_buffer;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    double evaluateImage() {
        cv::Mat frame = camera->getFrame();
        if (frame.empty()) return 0.0;
        return laplacianVariance(frame);
    }

    std::pair<int, double> focusing(int step, double threshold, int max_dec_count) {
        value_buffer.clear();
        int max_index = focuser->get(Focuser::OPT_FOCUS);
        double max_value = 0.0, last_value = -1.0;
        int dec_count = 0;

        int focal_distance = max_index;
        focuser->set(Focuser::OPT_FOCUS, focal_distance);
        while (true) {
            focuser->set(Focuser::OPT_FOCUS, focal_distance);
            double val = filteredValue(evaluateImage());
            if (debug) std::cout << "Value: " << val << ", Focal: " << focal_distance << std::endl;

            if (val > max_value) {
                max_value = val;
                max_index = focal_distance;
            }

            if (last_value - val > threshold) {
                dec_count++;
                if (debug) std::cout << "Decreasing trend. Count: " << dec_count << std::endl;
            } else if (last_value != val) {
                dec_count = 0;
            }

            if (dec_count > max_dec_count) break;

            last_value = val;
            focal_distance = focuser->get(Focuser::OPT_FOCUS) + step;
            if (focal_distance > MAX_FOCUS_VALUE) break;
        }
        return { max_index, max_value };
    }

    std::tuple<std::vector<double>, std::vector<int>, std::vector<double>>
    CoarseAdjustment(int st_point, int ed_point) {
        std::vector<cv::Mat> images;
        std::vector<int> index_list;
        std::vector<double> eval_list;
        std::vector<double> time_list;

        focuser->set(Focuser::OPT_FOCUS, st_point);
        auto start_time = std::chrono::steady_clock::now();
        time_list.push_back(0.0);
        images.push_back(camera->getFrame());

        focuser->set(Focuser::OPT_FOCUS, ed_point, 0);
        while (focuser->isBusy()) {
            auto now = std::chrono::steady_clock::now();
            time_list.push_back(std::chrono::duration<double>(now - start_time).count());
            images.push_back(camera->getFrame());
        }

        for (const auto& img : images) {
            double sharpness = laplacianVariance(img);
            eval_list.push_back(sharpness);
        }

        for (size_t i = 0; i < eval_list.size(); ++i) {
            index_list.push_back(static_cast<int>(i));
        }

        return { eval_list, index_list, time_list };
    }

    std::pair<int, double> startFocus() {
        auto start = std::chrono::steady_clock::now();
        focuser->reset(Focuser::OPT_FOCUS);
        MAX_FOCUS_VALUE = getEndPoint();
        focuser->set(Focuser::OPT_FOCUS, getStartingPoint());

        auto [max_index, max_value] = focusing(300, 1.0, 1);
        focuser->set(Focuser::OPT_FOCUS, max_index - 630);
        std::tie(max_index, max_value) = focusing(50, 1.0, 4);
        focuser->set(Focuser::OPT_FOCUS, max_index - 30);

        if (debug) {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            std::cout << "Focus time: " << duration.count() / 1000.0 << "s\n";
        }

        return { max_index, max_value };
    }

    std::pair<int, double> startFocus2() {
        focuser->reset(Focuser::OPT_FOCUS);
        MAX_FOCUS_VALUE = getEndPoint();
        int starting_point = getStartingPoint();

        auto [eval_list, index_list, time_list] = CoarseAdjustment(starting_point, MAX_FOCUS_VALUE);

        auto max_it = std::max_element(eval_list.begin(), eval_list.end());
        int max_index = static_cast<int>(std::distance(eval_list.begin(), max_it));
        double total_time = time_list.back();
        double max_time = time_list[max_index];
        int new_focus = static_cast<int>(((max_time / total_time) * (MAX_FOCUS_VALUE - starting_point)) + starting_point);
        focuser->set(Focuser::OPT_FOCUS, new_focus);

        auto [final_index, max_val] = focusing(50, 1.0, 4);
        focuser->set(Focuser::OPT_FOCUS, final_index - 30);

        if (debug) {
            std::cout << "Coarse+fine focus complete. Best: " << final_index
                      << ", Sharpness: " << max_val << std::endl;
        }

        return { final_index, max_val };
    }

private:
    Focuser* focuser;
    Camera* camera;
    std::vector<double> value_buffer;
    bool debug;
    int MAX_FOCUS_VALUE;
};

#endif // AUTOFOCUS_HPP
