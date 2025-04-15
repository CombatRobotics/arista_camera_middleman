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
    CONTROL_MODE,
    IDLE,
    ERROR,
    UNKNOWN
};

int main(int argc, char** argv) {
    // Register signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Default parameters
    const char* can_device = "can0";
    int can_bitrate = 500000;
    arista_camera_middleman::CanDevice can_device_handler(can_device, can_bitrate);
    const int _s = can_device_handler.get_socket_fd();
    setsockopt(_s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    can_frame frame; //
    while(1) {
        int nbytes = read(_s, &frame, sizeof(frame));
        if(nbytes > 0) {
            printf("can_id = 0x%X\r\ncan_dlc = %d \r\n", frame.can_id, frame.can_dlc);
            int i = 0;
            for(i = 0; i < 8; i++)
                printf("data[%d] = %d\r\n", i, frame.data[i]);
            break;
        }
    }
    // // No filter
    // if (!can_device_handler.clear_filter()) {
    //     std::cerr << "Failed to clear CAN filter" << std::endl;
    //     return -1;
    // }
    // Start state machine
    CanCommStates state = CanCommStates::UNINTIALIZED;

    while (g_running) {

        std::cout << "Current state: " << static_cast<int>(state) << std::endl;
        switch (state) {

            case CanCommStates::UNINTIALIZED:
                // Broadcast message
                {
                    arista_camera_middleman::protocol::TxData_t tx_data;
                    tx_data.setBroadcastKey();
                    can_frame can_data = tx_data.get_can_frame();
                    if (!can_device_handler.send_can_frame(&can_data)) {
                        std::cerr << "Failed to send CAN frame" << std::endl;
                        return -1;
                    }
                    state = CanCommStates::BROADCASTED;
                    break;
                }
            case CanCommStates::BROADCASTED:
                // Wait for ACK
                {
                    can_frame can_data;
                    printf("Waiting for ack\n");
                    if (!can_device_handler.receive_can_frame(&can_data)) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        return -1;
                    }
                    printf("ACK recieved : %x\n",can_data.can_id);
                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
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
                        return -1;
                    }
                    state = CanCommStates::WHO_DIS_SENT;
                    break;
                }
            case CanCommStates::WHO_DIS_SENT:
                // Wait for WHO_AM_I
                {
                    can_frame can_data;
                    if (!can_device_handler.receive_can_frame(&can_data)) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        return -1;
                    }

                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
                    if (function_id == arista_camera_middleman::protocol::RxData_t::FunctionId::WHO_AM_I_ACK) {
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
                        return -1;
                    }
                    state = CanCommStates::CALIB_QUERY_SENT;
                    break;
                }
            case CanCommStates::CALIB_QUERY_SENT:
                // Wait for CALIBRATION_ACK
                {
                    can_frame can_data;
                    if (!can_device_handler.receive_can_frame(&can_data)) {
                        std::cerr << "Failed to receive CAN frame" << std::endl;
                        return -1;
                    }

                    arista_camera_middleman::protocol::RxData_t::Data rx_data;
                    arista_camera_middleman::protocol::RxData_t::FunctionId function_id;
                    function_id = arista_camera_middleman::protocol::get_rx_data(can_data, rx_data);
                    if (function_id == arista_camera_middleman::protocol::RxData_t::FunctionId::CALLIBRATION_ACK) {
                        state = CanCommStates::CALIBRATION_RECVD;
                    }
                    else {
                        state = CanCommStates::ERROR;
                    }
                    break;
                }
            case CanCommStates::CALIBRATION_RECVD:
                // Send CONTROL_MODE
                {
                    std::cout << "Switching to control mode" << std::endl;
                    exit(EXIT_SUCCESS);
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
    
    return 0;
}