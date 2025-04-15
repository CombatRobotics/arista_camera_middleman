#include "arista_camera_middleman/RpiCamera.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto camera_node = std::make_shared<Camera>("/image");
    
    std::thread ros_spin([&]() {
        rclcpp::spin(camera_node);
    });

    // camera_node->start_preview();  // Start preview in a separate thread
    camera_node->blocking_preview();

    rclcpp::shutdown();
    ros_spin.join();
    return 0;
}
