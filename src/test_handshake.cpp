#include "arista_camera_middleman/CanDevice.hpp"
#include "arista_camera_middleman/CanProto.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <signal.h>
#include <string.h>
#include <sys/select.h>
#include <errno.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "rclcpp/rclcpp.hpp"
#include "arista_camera_middleman/PayloadCtrl.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"


// Global flag for graceful shutdown
volatile sig_atomic_t g_running = 1;

// Signal handler
void signal_handler(int signum) {
    g_running = 0;
    std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
}

enum class CanCommStates {
    UNINTIALIZED,
    BROADCASTED,
    ACK_RECVD,
    WHO_DIS_SENT,
    IDENTIFIED,
    CALIB_QUERY_SENT,
    CALIBRATION_RECVD,
    CALLIBRATION_CMD,
    CONTROL_MODE,
    IDLE,
    ERROR,
    UNKNOWN
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Register signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Default parameters
    const char* can_device = "can0";
    int can_bitrate = 500000;


    // Initialize CAN interface before creating the handler
    // std::string cmd1 = "sudo ip link set " + std::string(can_device) + " type can bitrate " + std::to_string(can_bitrate);
    // std::string cmd2 = "sudo ifconfig " + std::string(can_device) + " up";
    // system(cmd1.c_str());
    // system(cmd2.c_str());
    // std::cout << "CAN interface initialized" << std::endl;
    auto payload_ctrl_node = std::make_shared<arista_camera_middleman::PayloadControl>();
    arista_camera_middleman::CanDevice can_device_handler(can_device, can_bitrate);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(payload_ctrl_node);
    // // No filter
    // if (!can_device_handler.clear_filter()) {
    //     std::cerr << "Failed to clear CAN filter" << std::endl;
    //     return -1;
    // }
    // Start state machine
    CanCommStates state = CanCommStates::UNINTIALIZED;
    rclcpp::Rate loop_rate(100);

    while (g_running && rclcpp::ok()) {
        executor.spin_some(loop_rate.period());
        if (state!=CanCommStates::CONTROL_MODE){
            std::cout << "Current state: " << static_cast<int>(state) << std::endl;
        }
        switch (state) {

            case CanCommStates::UNINTIALIZED:
                // Broadcast message
                {
                    arista_camera_middleman::protocol::TxData_t tx_data;
                    tx_data.setBroadcastKey();
                    can_frame can_data = tx_data.get_can_frame();
                    if (!can_device_handler.send_can_frame(&can_data)) {
                        std::cerr << "Failed to send CAN frame" << std::endl;
                        break;
                    }
                    state = CanCommStates::BROADCASTED;
                    break;
                }
            case CanCommStates::BROADCASTED:
                // Wait for ACK
                {
                    can_frame can_data;
                    printf("Waiting for ack\n");
                    int ret = can_device_handler.recv_can_timeout(&can_data,30000);
                    if(ret==-2){
                        std::cerr << "Timeout" << std::endl;
                        state = CanCommStates::UNINTIALIZED;
                        break;
                    }
                    if (ret < 0) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        state = CanCommStates::BROADCASTED;
                        break;
                    }
                    printf("ACK recieved : %x\n",can_data.can_id);
                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
                    printf("FNID recieved : %x\n",function_id);
                    if (function_id == arista_camera_middleman::protocol::RxData_t::FunctionId::BROADCAST_ACK) {
                        state = CanCommStates::ACK_RECVD;
                    }
                    else {
                        state = CanCommStates::ERROR;
                    }
                    break;

                }
            case CanCommStates::ACK_RECVD:
                // Send WHO_AM_I
                {
                    arista_camera_middleman::protocol::TxData_t tx_data;
                    tx_data.setWhoAmI();
                    can_frame can_data = tx_data.get_can_frame();
                    if (!can_device_handler.send_can_frame(&can_data)) {
                        std::cerr << "Failed to send CAN frame" << std::endl;
                        state = CanCommStates::ACK_RECVD;
                        break;
                    }
                    state = CanCommStates::WHO_DIS_SENT;
                    break;
                }
            case CanCommStates::WHO_DIS_SENT:
                // Wait for WHO_AM_I
                {
                    can_frame can_data;
                    int ret = can_device_handler.recv_can_timeout(&can_data,300);
                    if(ret==-2){
                        std::cerr << "Timeout" << std::endl;
                        state = CanCommStates::ACK_RECVD;
                        break;
                    }
                    if (ret < 0) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        state = CanCommStates::ERROR;
                    }

                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
                    if (function_id == arista_camera_middleman::protocol::RxData_t::FunctionId::WHO_AM_I_ACK) {
                        std::cout << "Received WHO_AM_I_ACK" << std::endl;
                        if (rx_data.device_id.device_id == arista_camera_middleman::protocol::device_id_t::GIMBAL)
                        {
                            std::cout << "Identified as gimbal" << std::endl;
                            payload_ctrl_node->set_payload_type(arista_camera_middleman::PayloadType::GIMBAL);
                        } else if (rx_data.device_id.device_id == arista_camera_middleman::protocol::device_id_t::TURRET) {
                            std::cout << "Identified as turret" << std::endl;
                            payload_ctrl_node->set_payload_type(arista_camera_middleman::PayloadType::TURRET);
                        } else {
                            std::cout << "Identified as unknown" << std::endl;
                            state = CanCommStates::ERROR;
                            break;
                        }
                        
                        state = CanCommStates::IDENTIFIED;
                    }
                    else {
                        state = CanCommStates::ERROR;
                    }
                    break;
                }
            case CanCommStates::IDENTIFIED:
                // Send CALIBRATION_QUERY
                {
                    arista_camera_middleman::protocol::TxData_t tx_data;
                    tx_data.setCallibrationStatus();
                    can_frame can_data = tx_data.get_can_frame();
                    if (!can_device_handler.send_can_frame(&can_data)) {
                        std::cerr << "Failed to send CAN frame" << std::endl;
                        state = CanCommStates::IDENTIFIED;
                        break;
                    }
                    state = CanCommStates::CALIB_QUERY_SENT;
                    break;
                }
            case CanCommStates::CALIB_QUERY_SENT:
                // Wait for CALIBRATION_ACK
                {
                    can_frame can_data;
                    int ret = can_device_handler.recv_can_timeout(&can_data,300);
                    if (ret == -2) {
                        std::cerr << "Timeout" << std::endl;
                        state = CanCommStates::IDENTIFIED;
                        break;
                    }
                    if (ret < 0) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        state = CanCommStates::ERROR;
                        break;
                    }

                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
                    if (function_id == arista_camera_middleman::protocol::RxData_t::FunctionId::CALLIBRATION_ACK) {
                        if(rx_data.callibration.status){
                            std::cout << "Callibration Completed" << std::endl;
                            auto& pan_home = rx_data.callibration.pan_config.home_position;
                            auto& tilt_home = rx_data.callibration.tilt_config.home_position;
                            arista_interfaces::msg::GimbalPos range_msg;
                            range_msg.yaw = rx_data.callibration.pan_config.range;
                            range_msg.pitch = rx_data.callibration.tilt_config.range;
                            payload_ctrl_node->_initialize_ros();
                            payload_ctrl_node->set_range(range_msg);
                            printf("Pan Home: %f, Tilt Home: %f\n", pan_home, tilt_home);
                            printf("Pan Range: %f, Tilt Range: %f\n", range_msg.yaw, range_msg.pitch);
                            state = CanCommStates::CALIBRATION_RECVD;
                            break;
                        } else {
                            std::cout << "Callibration Failed, Retrying" << std::endl;
                            state = CanCommStates::CALLIBRATION_CMD;
                            break;
                        }
                    }
                    else {
                        state = CanCommStates::ERROR;
                    }
                    break;
                }
            case CanCommStates::CALLIBRATION_CMD:
                // Send CALIBRATION_QUERY
                {
                    arista_camera_middleman::protocol::TxData_t tx_data;
                    tx_data.setCallibrationCmd();
                    can_frame can_data = tx_data.get_can_frame();
                    if (!can_device_handler.send_can_frame(&can_data)) {
                        std::cerr << "Failed to send CAN frame" << std::endl;
                        state = CanCommStates::CALLIBRATION_CMD;
                        break;
                    }
                    state = CanCommStates::CALIB_QUERY_SENT;
                    break;
                }
            case CanCommStates::CALIBRATION_RECVD:
                // Send CONTROL_MODE
                {

                    std::cout << "Switching to control mode" << std::endl;
                    state = CanCommStates::CONTROL_MODE;
                    break;
                }
            case CanCommStates::CONTROL_MODE:
                // Send CONTROL_MODE
                {   
                    arista_interfaces::msg::GimbalPos ctrl_msg;
                    if(payload_ctrl_node->pop_ctrl_cmd(ctrl_msg)){
                        arista_camera_middleman::protocol::TxData_t tx_data;
                        arista_camera_middleman::AngleCmd_t pan_cmd,tilt_cmd;
                        tx_data.setSpeedData(ctrl_msg.yaw,ctrl_msg.pitch);
                        can_frame can_data = tx_data.get_can_frame();
                        if (!can_device_handler.send_can_frame(&can_data)) {
                            std::cerr << "Failed to send CAN frame" << std::endl;
                            state = CanCommStates::ERROR;
                            break;
                        }
                        break;
                        // tx_data.setControlMode();
                        // pan_cmd = arista_camera_middleman::angle2cmd(ctrl_msg.yaw);
                        // tilt_cmd = arista_camera_middleman::angle2cmd(ctrl_msg.pitch);
                        // tx_data.setControlData(pan_cmd,tilt_cmd);
                        // can_frame can_data = tx_data.get_can_frame();
                        // if (!can_device_handler.send_can_frame(&can_data)) {
                        //     std::cerr << "Failed to send CAN frame" << std::endl;
                        //     state = CanCommStates::ERROR;
                        //     break;
                        // }
                        // break;
                    }
                    if(payload_ctrl_node->should_trigger()){
                        arista_camera_middleman::protocol::TxData_t tx_data;
                        // tx_data.setTrigger();
                        can_frame can_data = tx_data.get_can_frame();
                        if (!can_device_handler.send_can_frame(&can_data)) {
                            std::cerr << "Failed to send CAN frame" << std::endl;
                            state = CanCommStates::ERROR;
                            break;
                        }
                    }
                    break;
                    
                }
            case CanCommStates::ERROR:
                std::cerr << "Error in state machine" << std::endl;
                state = CanCommStates::UNINTIALIZED;
                break;
            default:
                std::cerr << "Unknown state" << std::endl;
                g_running = false;
                break;
        }
        
    } 
    
    std::cout << "Shutting down..." << std::endl;
    
    // Clean up CAN interface
    std::string cmd_down = "sudo ifconfig " + std::string(can_device) + " down";
    system(cmd_down.c_str());
    
    return 0;
}