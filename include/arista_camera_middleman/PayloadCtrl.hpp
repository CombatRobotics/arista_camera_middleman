#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
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
        using CbQueue = FixedQueue<GimbalPos, 2>;
        PayloadControl(): Node("payload_control"){};
        inline void set_payload_type(PayloadType type){payload_type = type;}
        inline void set_range(arista_interfaces::msg::GimbalPos& range){
            payload_range = range;
            max_pose_pub_->publish(range);
            }
        bool pop_ctrl_cmd(GimbalPos& ctrl_cmd){
            if(!cb_queue.empty()){
                ctrl_cmd = cb_queue.front();
                payload_pos = ctrl_cmd;
                cb_queue.pop();
                return true;
            }
            return false;
        }
        void _initialize_ros(){
            if(_initialized_ros) return;
            gimbal_ctrl_sub = this->create_subscription<GimbalPos>(
                "gimbal/cmd_pose", 10,
                std::bind(&PayloadControl::gimbal_ctrl_callback, this, std::placeholders::_1)
            );
            turret_ctrl_sub = this->create_subscription<std_msgs::msg::UInt32>(
                "gimbal/trigger", 10,
                std::bind(&PayloadControl::turret_ctrl_callback, this, std::placeholders::_1)
            );
            max_pose_pub_ = this->create_publisher<GimbalPos>("gimbal/max_pose", 10);
            current_pose_pub_ = this->create_publisher<GimbalPos>("gimbal/current_pose", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000), std::bind(&PayloadControl::timer_callback, this));
            _initialized_ros = true;
        };
        void _shutdown_ros(){
            if(!_initialized_ros) return;
            gimbal_ctrl_sub.reset();
            turret_ctrl_sub.reset();
            max_pose_pub_.reset();
            current_pose_pub_.reset();
            timer_.reset();
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

        private:
        void gimbal_ctrl_callback(const GimbalPos::SharedPtr msg){
            GimbalPos msg_copy = *msg;
            if((msg->yaw)>(payload_range.yaw)){
                msg_copy.yaw = payload_range.yaw;
            } else if ((msg->yaw)<(0)){
                msg_copy.yaw = 0;
            }
            if((msg->pitch)>(payload_range.pitch)){
                msg_copy.pitch = payload_range.pitch;
            } else if ((msg->pitch)<(0)){
                msg_copy.pitch = 0;
            }
            cb_queue.push(*msg);
        }
        void turret_ctrl_callback(const std_msgs::msg::UInt32::SharedPtr msg){
            if(msg->data != _trigger_seq){
                _trigger_count = (msg->data)-_trigger_seq;
            }
            _trigger_seq = msg->data;

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
        rclcpp::Subscription<GimbalPos>::SharedPtr gimbal_ctrl_sub;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr turret_ctrl_sub;
        rclcpp::Publisher<GimbalPos>::SharedPtr max_pose_pub_,current_pose_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}