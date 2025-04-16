#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "arista_interfaces/msg/gimbal_ctrl.hpp"
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
        using GimbalCtrl = arista_interfaces::msg::GimbalCtrl;
        using CbQueue = FixedQueue<GimbalCtrl, 2>;
        PayloadControl(): Node("payload_control"){};
        inline void set_payload_type(PayloadType type){payload_type = type;}
        inline void set_range(arista_interfaces::msg::GimbalPos& range){payload_range = range;}
        bool pop_ctrl_cmd(GimbalCtrl& ctrl_cmd){
            if(!cb_queue.empty()){
                ctrl_cmd = cb_queue.front();
                cb_queue.pop();
                return true;
            }
            return false;
        }
        void _initialize_ros(){
            if(_initialized_ros) return;
            gimbal_ctrl_sub = this->create_subscription<GimbalCtrl>(
                "/gimbal_ctrl", 10,
                std::bind(&PayloadControl::gimbal_ctrl_callback, this, std::placeholders::_1)
            );
            turret_ctrl_sub = this->create_subscription<std_msgs::msg::Empty>(
                "/trigger", 10,
                std::bind(&PayloadControl::turret_ctrl_callback, this, std::placeholders::_1)
            );
            _initialized_ros = true;
        };
        void _shutdown_ros(){
            if(!_initialized_ros) return;
            gimbal_ctrl_sub.reset();
            turret_ctrl_sub.reset();
            _initialized_ros = false;
        }
        private:
        void gimbal_ctrl_callback(const GimbalCtrl::SharedPtr msg){
            cb_queue.push(*msg);
        }
        void turret_ctrl_callback(const std_msgs::msg::Empty::SharedPtr msg){
            // trigger_count++;
        };
        PayloadType payload_type;
        arista_interfaces::msg::GimbalPos payload_pos,payload_range;
        CbQueue cb_queue;
        bool _trigger_set;
        bool _initialized_ros = false;
        rclcpp::Subscription<GimbalCtrl>::SharedPtr gimbal_ctrl_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr turret_ctrl_sub;
    };
}