#pragma once

#include <cstdint>
#include <linux/can.h>
#include <string.h>

namespace arista_camera_middleman
{
namespace protocol
{

enum class device_id_t {
    CAMERA = 0x13,
    GIMBAL = 0x23,
};

enum class payload_type_t {
    ACK = 0x01,
    NACK = 0x00,
};
static constexpr uint8_t SENDER_CAN_ID = 0x39;
#define PAYLOAD_ACK 0x01
#define PAYLOAD_NACK 0x00

//function_ids
enum class gimbal_rx_pkt_t {
    BROADCAST_ACK             = 0x20,
    CALLIBRATION_ACK          = 0x21,
    ALIVE_PKT                 = 0x22,
    FEEBACK_PKT               = 0x23,
    WHO_AMI_ACK               = 0x33,
} ;

enum class gimbal_tx_pkt_t {
    BROADCASTING_PERIPHERAL = 0x24,
    HEARTBEAT_PKT           = 0x25,
    INCREMENTAL_PKT         = 0x26,
    ANGLE_PKT               = 0x27,
    FEEDBACK_REQUEST        = 0x28,
    STOP_PKT                = 0x29,
    SPEED_MODE_PKT          = 0x30,
    CALLIBRATION_CMD        = 0x31,
    WHO_DIS_PKT             = 0x32,
    callibration     = 0x36,

} ;

constexpr uint8_t MAX_PAYLOAD_SIZE = 8;
// struct payload_identifer_t
// {
//     uint8_t function_id;
//     uint8_t sender_id;
// };

// struct can_frame_t {
//     payload_identifer_t payload_id;
//     uint8_t payload[MAX_PAYLOAD_SIZE];
// };

using angle_t = double;
using gimbal_position_t = angle_t;
using gimbal_pos_mapped_t = uint16_t;

struct CanIdentifier{
    uint8_t sender_id;
    uint8_t function_id;
};

struct gimbal_axis_config_t {
    gimbal_position_t range;
    gimbal_position_t home_position;
};

const long Broadcast_Key = 0x435249504C343536;
constexpr angle_t MAX_ANGLE = 360.0;
constexpr gimbal_pos_mapped_t MAX_POS_MAPPED = 0x10000;


CanIdentifier get_can_identifier(const canid_t& can_id_data){
    const uint8_t* can_data_buff = (const uint8_t* )&can_id_data;
    CanIdentifier id;
    id.sender_id = can_data_buff[0];
    id.function_id = can_data_buff[1];
    return id;
};





struct RxData_t{
    enum class FunctionId {
        BROADCAST_ACK             = 0x20,
        CALLIBRATION_ACK          = 0x21,
        ALIVE_PKT                 = 0x22,
        WHO_AM_I_ACK              = 0x33,
        FEEBACK_PKT               = 0x23,
        UNKNOWN_PKT               = 0x00
    } function_id;
    union Data {
        struct Broadcast{
            payload_type_t status;
        } broadcast;
        struct Callibration{
            gimbal_axis_config_t pan_config;
            gimbal_axis_config_t tilt_config;
        } callibration;
        struct Alive{
            bool status;
        } is_alive;
        struct device_id{
            device_id_t device_id;
        } device_id;
        struct Feedback{
            gimbal_position_t pan_position;
            gimbal_position_t tilt_position;
        } feedback;

    } data;
};


struct TxData_t{
    enum class FunctionId {
        BROADCAST = 0x24,
        HEARTBEAT_PKT           = 0x25,
        INCREMENTAL_PKT         = 0x26,
        ANGLE_PKT               = 0x27,
        FEEDBACK_REQUEST        = 0x28,
        STOP_PKT                = 0x29,
        SPEED_MODE_PKT          = 0x30,
        CALLIBRATION_CMD        = 0x31,
        WHO_DIS_PKT             = 0x32,
        callibration     = 0x36,
    } function_id;
    union Data {
        struct BroadcastKey{
            long key;
        } broadcast_key;
        struct Heartbeat{
            payload_type_t status;
        } heartbeat;
        struct Incremental{
            payload_type_t status;
            gimbal_position_t pan_position;
            gimbal_position_t tilt_position;
        } incremental;
        struct Angle{
            payload_type_t status;
            gimbal_position_t pan_position;
            gimbal_position_t tilt_position;
        } angle;
        struct FeedbackRequest{
            payload_type_t status;
        } feedback_request;
        struct Stop{
            payload_type_t status;
        } stop;
        struct SpeedMode{
            payload_type_t status;
        } speed_mode;
        struct Callibration{
            payload_type_t status;
        } callibration;
        struct WhoAmI{} who_am_i;
        struct CallibrationStatus{
        } callibration_status;
    } data;
    void setBroadcastKey(){
        function_id = FunctionId::BROADCAST;
        data.broadcast_key = {Broadcast_Key};
    }
    void setWhoAmI(){
        function_id = FunctionId::WHO_DIS_PKT;
    }
    void setCallibrationStatus(){
        function_id = FunctionId::callibration;    
    }
    canid_t get_can_id()const{
        canid_t can_id_data;
        uint8_t* can_data_buff = (uint8_t* )&can_id_data;
        can_data_buff[0] = SENDER_CAN_ID;
        can_data_buff[1] = static_cast<uint8_t>(function_id);
        can_id_data |= CAN_EFF_FLAG;
        printf("can_id : %x \n", can_id_data);
        return can_id_data;
    }
    can_frame get_can_frame() const {
        can_frame frame;
        frame.can_id = get_can_id();
        frame.can_dlc = MAX_PAYLOAD_SIZE;
        memset(frame.data, 0, MAX_PAYLOAD_SIZE);
        switch (function_id)
        {
        case FunctionId::BROADCAST:
            {
                memcpy(frame.data, &data.broadcast_key.key, sizeof(data.broadcast_key.key));
                break;
            }
        case FunctionId::WHO_DIS_PKT:
            {
                break;
            }
        case FunctionId::callibration:
            {
                // gimbal_pos_mapped_t pan_range;
                // memcpy(&pan_range, &data.callibration.pan_config.range, sizeof(data.callibration.pan_config.range));
                // gimbal_pos_mapped_t tilt_range;
                // memcpy(&tilt_range, &data.callibration.tilt_config.range, sizeof(data.callibration.tilt_config.range));
                // data.callibration.pan_config.range = convert_gimbal_pos_mapped_to_angle(pan_range);
                // data.callibration.tilt_config.range = convert_gimbal_pos_mapped_to_angle(tilt_range);
                // gimbal_pos_mapped_t pan_home;
                // memcpy(&pan_home, &data.callibration.pan_config.home_position, sizeof(data.callibration.pan_config.home_position));
                // gimbal_pos_mapped_t tilt_home;
                // memcpy(&tilt_home, &data.callibration.tilt_config.home_position, sizeof(data.callibration.tilt_config.home_position));
                // data.callibration.pan_config.home_position = convert_gimbal_pos_mapped_to_angle(pan_home);
                // data.callibration.tilt_config.home_position = convert_gimbal_pos_mapped_to_angle(tilt_home);
                break;
            }
        }
        return frame;
    }
};

gimbal_position_t convert_gimbal_pos_mapped_to_angle(gimbal_pos_mapped_t pos_mapped){
    return static_cast<gimbal_position_t>((pos_mapped*MAX_ANGLE) / MAX_POS_MAPPED);
}

gimbal_pos_mapped_t convert_gimbal_angle_to_pos_mapped(gimbal_position_t angle){
    return static_cast<gimbal_pos_mapped_t>((angle*MAX_POS_MAPPED) / MAX_ANGLE);
}


RxData_t::FunctionId get_function_id(const canid_t& can_id_data){
    CanIdentifier id = get_can_identifier(can_id_data);
    return static_cast<RxData_t::FunctionId>(id.function_id);
}
RxData_t::FunctionId get_rx_data(const can_frame& can_data, RxData_t::Data& data){
    RxData_t::FunctionId function_id = get_function_id(can_data.can_id);
    switch(function_id){
        case RxData_t::FunctionId::BROADCAST_ACK:
            {
                data.broadcast.status = static_cast<payload_type_t>(can_data.data[0]);
                return function_id;
            }
        case RxData_t::FunctionId::CALLIBRATION_ACK:
            {
                gimbal_pos_mapped_t pan_range;
                memcpy(&pan_range, &data.callibration.pan_config.range, sizeof(data.callibration.pan_config.range));
                gimbal_pos_mapped_t tilt_range;
                memcpy(&tilt_range, &data.callibration.tilt_config.range, sizeof(data.callibration.tilt_config.range));
                data.callibration.pan_config.range = convert_gimbal_pos_mapped_to_angle(pan_range);
                data.callibration.tilt_config.range = convert_gimbal_pos_mapped_to_angle(tilt_range);
                gimbal_pos_mapped_t pan_home;
                memcpy(&pan_home, &data.callibration.pan_config.home_position, sizeof(data.callibration.pan_config.home_position));
                gimbal_pos_mapped_t tilt_home;
                memcpy(&tilt_home, &data.callibration.tilt_config.home_position, sizeof(data.callibration.tilt_config.home_position));
                data.callibration.pan_config.home_position = convert_gimbal_pos_mapped_to_angle(pan_home);
                data.callibration.tilt_config.home_position = convert_gimbal_pos_mapped_to_angle(tilt_home);
                
                return function_id;
            }
        case RxData_t::FunctionId::ALIVE_PKT:
            {
                data.is_alive.status = static_cast<bool>(can_data.data[0]);
                return function_id;
            }
        case RxData_t::FunctionId::FEEBACK_PKT:
            {

                return function_id;
            }
        default:
            return RxData_t::FunctionId::BROADCAST_ACK;
    }

};
//Predefined payloads
//HOME_POSITION PAYLOAD



//Handshake Sequence
//Broadcast -> ACK
//WHO_DIS -> ME_DIS 
//CALIB_STATUS -> ack/nack

// Handshake packet structure




} // namespace protocol
} // namespace arista_camera_middleman