#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "arista_interfaces/msg/gimbal_pos.hpp"

using namespace std::chrono_literals;

class GimbalStabilizer : public rclcpp::Node
{
public:
    using GimbalPos = arista_interfaces::msg::GimbalPos;

    GimbalStabilizer() : Node("gimbal_stabilizer")
    {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("input_topic", "gimbal/cmd_pose");
        this->declare_parameter<std::string>("output_topic", "gimbal/cmd_pose/stable");
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<int>("timeout_ms", 500);

        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();

        // Initialize stored data to zero
        stored_data_.pitch = 0.0;
        stored_data_.yaw = 0.0;
        last_received_time_ = this->now();

        // Create subscriber
        subscription_ = this->create_subscription<GimbalPos>(
            input_topic_, 10,
            std::bind(&GimbalStabilizer::topic_callback, this, std::placeholders::_1));

        // Create publisher
        publisher_ = this->create_publisher<GimbalPos>(output_topic_, 10);

        // Create timer for publishing at specified rate
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&GimbalStabilizer::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "GimbalStabilizer started");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "  Timeout: %d ms", timeout_ms_);
    }

private:
    void topic_callback(const GimbalPos::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        stored_data_ = *msg;
        last_received_time_ = this->now();
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Check if timeout has occurred
        auto time_since_last = (this->now() - last_received_time_).nanoseconds() / 1e6;  // Convert to ms

        GimbalPos output_msg;
        if (time_since_last > timeout_ms_) {
            // Timeout - set to zero
            output_msg.pitch = 0.0;
            output_msg.yaw = 0.0;
        } else {
            // Use stored data
            output_msg = stored_data_;
        }

        publisher_->publish(output_msg);
    }

    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    int timeout_ms_;

    // Data storage
    GimbalPos stored_data_;
    rclcpp::Time last_received_time_;
    std::mutex data_mutex_;

    // ROS interfaces
    rclcpp::Subscription<GimbalPos>::SharedPtr subscription_;
    rclcpp::Publisher<GimbalPos>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalStabilizer>());
    rclcpp::shutdown();
    return 0;
}
