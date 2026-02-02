#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "arista_interfaces/msg/gimbal_pos.hpp"
#include <vector>
#include <queue>
#include <deque>
#include <iostream>


namespace arista_camera_middleman {

    template <typename T, int MaxLen, typename Container=std::deque<T>>
    class FixedQueue : public std::queue<T, Container> {
    public:
        void push(const T& value) {
            if (this->size() == MaxLen) {
            this->c.pop_front();
            }
            std::queue<T, Container>::push(value);
        }
    };




    enum class PayloadType {
        UNKOWN = 0,
        TURRET = 1,
        GIMBAL = 2
    };
    
    class PayloadControl: public rclcpp::Node {
        public:
        using GimbalPos = arista_interfaces::msg::GimbalPos;
        static constexpr size_t MAX_QUEUE_LEN = 19;
        using CbQueue = FixedQueue<GimbalPos, MAX_QUEUE_LEN>;
        PayloadControl(): Node("payload_control"){};
        ~PayloadControl(){_shutdown_ros();}
        inline void set_payload_type(PayloadType type){payload_type = type;}
        inline void set_range(arista_interfaces::msg::GimbalPos& range){
            payload_range = range;
            max_pose_pub_->publish(range);
            }
        bool pop_ctrl_cmd(GimbalPos& ctrl_cmd){
            // printf("pop_ctrl_cmd qc=%u\n",_gimbal_req_count);
            if(!cb_queue.empty()){
                _gimbal_req_count = 0;
                _gimbal_stopped = false;
                ctrl_cmd = cb_queue.front();
                payload_pos = ctrl_cmd;
                cb_queue.pop();
                return true;
            } else if(!_gimbal_stopped){
                _gimbal_req_count++;
                if (_gimbal_req_count >= MAX_QUEUE_LEN){
                    _gimbal_stopped = true;
                    ctrl_cmd.yaw = 0;
                    ctrl_cmd.pitch = 0;
                    return true;
                }
            }
            return false;
        }
        void _initialize_ros(){
            if(_initialized_ros) return;
            gimbal_ctrl_sub = this->create_subscription<GimbalPos>(
                "gimbal/cmd_pose/stable", 10,
                std::bind(&PayloadControl::gimbal_ctrl_callback, this, std::placeholders::_1)
            );
            turret_ctrl_sub = this->create_subscription<std_msgs::msg::UInt16>(
                "gimbal/trigger", 10,
                std::bind(&PayloadControl::turret_ctrl_callback, this, std::placeholders::_1)
            );
            auto_mode_sub = this->create_subscription<std_msgs::msg::Bool>(
                "gimbal/auto_mode", 10,
                std::bind(&PayloadControl::auto_mode_callback, this, std::placeholders::_1)
            );
            trigger_held_sub = this->create_subscription<std_msgs::msg::Bool>(
                "gimbal/trigger_held", 10,
                std::bind(&PayloadControl::trigger_held_callback, this, std::placeholders::_1)
            );
            max_pose_pub_ = this->create_publisher<GimbalPos>("gimbal/max_pose", 10);
            current_pose_pub_ = this->create_publisher<GimbalPos>("gimbal/current_pose", 10);
            encoder_pub_ = this->create_publisher<GimbalPos>("gimbal/encoder", 10);
            // timer_ = this->create_wall_timer(
            //     std::chrono::milliseconds(1000), std::bind(&PayloadControl::timer_callback, this));
            executor_.add_node(shared_from_this());
            executor_thread = std::thread([this]() {
                executor_.spin();
            });
            _initialized_ros = true;
        };
        void _shutdown_ros(){
            if(!_initialized_ros) return;
            gimbal_ctrl_sub.reset();
            turret_ctrl_sub.reset();
            auto_mode_sub.reset();
            trigger_held_sub.reset();
            max_pose_pub_.reset();
            current_pose_pub_.reset();
            encoder_pub_.reset();
            timer_.reset();
            // executor_.remove_node(shared_from_this());
            executor_.cancel();
            _initialized_ros = false;
        }

        bool should_trigger(){
            if(payload_type == PayloadType::TURRET){
                if(_trigger_count !=0){
                    _trigger_count--;
                    return true;
                } else {
                    return false;
                }
            }
             return false;
        }

        bool is_auto_mode(){
            return _auto_mode_enabled;
        }

        bool is_trigger_held(){
            return _trigger_held;
        }

        bool get_auto_trigger_state(bool& is_firing){
            if(payload_type == PayloadType::TURRET && _auto_mode_enabled){
                is_firing = _trigger_held;
                return true;
            }
            return false;
        }

        void update_pan_encoder(double angle_degrees){
            encoder_data_.yaw = angle_degrees;
        }

        void update_tilt_encoder(double angle_degrees){
            encoder_data_.pitch = angle_degrees;
        }

        void publish_encoder_data(){
            if(encoder_pub_){
                encoder_pub_->publish(encoder_data_);
            }
        }

        void set_calibration_limits(double pan_min, double pan_max, double tilt_min, double tilt_max){
            _yaw_min = pan_min;
            _yaw_max = pan_max;
            _pitch_min = tilt_min;
            _pitch_max = tilt_max;
            _limits_received = true;
            printf("Calibration limits set: yaw=[%.2f, %.2f], pitch=[%.2f, %.2f]\n",
                   _yaw_min, _yaw_max, _pitch_min, _pitch_max);
        }

        private:
        void gimbal_ctrl_callback(const GimbalPos::SharedPtr msg){
            GimbalPos msg_copy = *msg;
            if(payload_type==PayloadType::TURRET){
                // Check limits if received - block velocity if at limit and trying to go further
                if(_limits_received){
                    // Yaw limits
                    if ((encoder_data_.yaw >= _yaw_max && msg->yaw > 0) ||
                        (encoder_data_.yaw <= _yaw_min && msg->yaw < 0)) {
                        msg_copy.yaw = 0;
                    } else {
                        msg_copy.yaw *= 4000;
                    }
                    // Pitch limits
                    if ((encoder_data_.pitch >= _pitch_max && msg->pitch > 0) ||
                        (encoder_data_.pitch <= _pitch_min && msg->pitch < 0)) {
                        msg_copy.pitch = 0;
                    } else {
                        msg_copy.pitch *= 4000;
                    }
                } else {
                    msg_copy.yaw *= 4000;
                    msg_copy.pitch *= 4000;
                }
                cb_queue.push(msg_copy);
            } else if (payload_type==PayloadType::GIMBAL) {
                msg_copy.yaw*=40;
                msg_copy.pitch*=-8;
                cb_queue.push(msg_copy);
            }
        }
        void turret_ctrl_callback(const std_msgs::msg::UInt16::SharedPtr msg){
            if(!_triiger_initialized){
                _trigger_seq = msg->data;
                _triiger_initialized = true;
            }
            if(msg->data != _trigger_seq){
                _trigger_count = (msg->data)-_trigger_seq;
            }
            _trigger_seq = msg->data;
        };
        void auto_mode_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if(_auto_mode_enabled && !msg->data){
                // Exiting auto mode - clear trigger count to prevent unwanted single shot
                _trigger_count = 0;
            }
            _auto_mode_enabled = msg->data;
        };
        void trigger_held_callback(const std_msgs::msg::Bool::SharedPtr msg){
            _trigger_held = msg->data;
        };
        void timer_callback(){
            if(_initialized_ros){
                current_pose_pub_->publish(payload_pos);
            }
        }
        PayloadType payload_type;
        arista_interfaces::msg::GimbalPos payload_pos,payload_range;
        CbQueue cb_queue;
        uint32_t _trigger_seq,_trigger_count = 0;
        bool _initialized_ros = false;
        uint32_t _gimbal_req_count = 0;
        bool _triiger_initialized=false;
        bool _gimbal_stopped = true;
        bool _auto_mode_enabled = false;
        bool _trigger_held = false;
        bool _limits_received = false;
        double _yaw_min = 0, _yaw_max = 0;
        double _pitch_min = 0, _pitch_max = 0;
        rclcpp::Subscription<GimbalPos>::SharedPtr gimbal_ctrl_sub;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr turret_ctrl_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_mode_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_held_sub;
        rclcpp::Publisher<GimbalPos>::SharedPtr max_pose_pub_,current_pose_pub_,encoder_pub_;
        GimbalPos encoder_data_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::thread executor_thread;
    };
}