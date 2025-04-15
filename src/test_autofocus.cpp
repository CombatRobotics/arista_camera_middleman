#include "arista_camera_middleman/RpiCamera.hpp"
#include "arista_camera_middleman/CamFocuser.hpp"
#include "arista_camera_middleman/AutoFocus.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create camera node and focuser
    auto camera_node = std::make_shared<Camera>("/image");
    Focuser focuser(1);  // I2C bus number, usually 1 on Raspberry Pi

    // Start ROS spin thread
    std::thread ros_spin_thread([&]() {
        rclcpp::spin(camera_node);
    });

    // Start preview in separate thread (uses OpenCV window)
    camera_node->start_preview();

    // Allow a short delay to accumulate frames
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Run autofocus
    AutoFocus auto_focus(&focuser, camera_node.get());
    auto_focus.setDebug(true);
    auto [focus_index, clarity_value] = auto_focus.startFocus();

    std::cout << "Autofocus complete. Best focus index: " << focus_index
              << ", Sharpness: " << clarity_value << std::endl;

    // Optional: Wait to view the focused image
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Clean shutdown
    camera_node->stop_preview();
    rclcpp::shutdown();
    ros_spin_thread.join();

    return 0;
}
