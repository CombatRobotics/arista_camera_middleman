#pragma once

#include <cstdint>
#include <linux/can.h>
#include <string.h>

namespace arista_camera_middleman
{
    using AngleCmd_t = uint16_t;
    using Angle_t = double;

template <typename T_in,typename T_out,bool en_map_store>
struct _MapStore {
    T_in min_in,max_in;
    T_out min_out,max_out;
    inline _MapStore(T_in _min_i,T_in _max_i,T_out _min_o,T_out _max_o){
        if(_min_i<=_max_i){
            min_in = _min_i;
            min_out = _min_o;
            if(en_map_store){
            max_in = _max_i;
            max_out = _max_o;
        }
        } else {
            min_in = _max_i;
            min_out = _max_o;
            if(en_map_store){
                max_in = _min_i;
                max_out = _min_o;
            }
        }
        
    }
};

template <typename T_in,typename T_out>
struct _MapStore<T_in, T_out,false> { 
    T_in min_in;
    T_out min_out;
    inline _MapStore(T_in _min_i,T_out _min_o){
        min_in = _min_i;
        min_out = _min_o;
    }
};
template <typename T>
T safe_div(T num,T denom,T def){
    if(denom!=0.0) return num/denom;
    else return def;
};

template <typename T_in,typename T_out,typename mul_T,bool clamp2edge>
struct LinearMapper
{
    inline LinearMapper(T_in min_input,T_in max_input,T_out min_output,T_out max_output):
    multiplier(safe_div<mul_T>(max_output-min_output,max_input-min_input,0.0)),
    _store(min_input,max_input,min_output,max_output)
    {};
    inline T_out get(T_in input) const {
        printf("input: %f\n",input);

        if(input<=_store.min_in) return _store.min_out;
        if(input>=_store.max_in) return _store.max_out;
        
        return (input - _store.min_in)*(multiplier) + _store.min_out;
    }
    private:
    const _MapStore<T_in,T_out,clamp2edge> _store;
    const mul_T multiplier;
};

template <typename T_in,typename T_out,typename mul_T>
struct LinearMapper<T_in,T_out,mul_T,false>
{
    inline LinearMapper(T_in min_input,T_in max_input,T_out min_output,T_out max_output):
    multiplier(mul_T((max_output-min_output))/mul_T((max_input-min_input))),
    _store(min_input,min_output)
    {};
    inline T_out get(T_in input) const {
        return ((input - _store.min_in)*multiplier) + _store.min_out;
    }
    private:
    const _MapStore<T_in,T_out,false> _store;
    const mul_T multiplier;
};


// using Angle2CmdMapper = LinearMapper<Angle_t, AngleCmd_t, double, true>;
// using Cmd2AngleMapper = LinearMapper<AngleCmd_t, Angle_t, double, true>;

// const arista_camera_middleman::Angle2CmdMapper angle2cmd_mapper(0.0, 360.0, 0, 0x10000U);
// const arista_camera_middleman::Cmd2AngleMapper cmd2angle_mapper(0U, 0x10000U, 0.0, 360.0);


AngleCmd_t angle2cmd(const Angle_t& angle){
    if(angle < 0.0){
        return 0;
    }
    if (angle> 360.0){
        return 0x10000U;
    }
    return ((angle*(0x10000U))/360.0);
}
Angle_t cmd2angle(const AngleCmd_t& angle_cmd){
    return ((angle_cmd*360.0)/(0x10000U));
}


namespace protocol
{

enum class device_id_t {
    GIMBAL = 0x13,
    TURRET = 0x23,
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
using gimbal_speed_t = angle_t;
using gimbal_rpm_t = uint16_t; 
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
            bool status;
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
        BROADCAST               = 0x24,
        HEARTBEAT_PKT           = 0x25,
        INCREMENTAL_PKT         = 0x26,
        ANGLE_PKT               = 0x27,
        FEEDBACK_REQUEST        = 0x28,
        STOP_PKT                = 0x29,
        SPEED_MODE_PKT          = 0x30,
        CALLIBRATION_CMD        = 0x31,
        WHO_DIS_PKT             = 0x32,
        CALLIBRATION_STATUS     = 0x36,
        TRIGGER_PKT            = 0x34,
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
        struct Speed{
            gimbal_rpm_t yaw,pitch;
            uint8_t yaw_dir,pitch_dir;
        } speed;
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
        struct Trigger{} trigger;
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
        function_id = FunctionId::CALLIBRATION_STATUS;    
    }
    void setCallibrationCmd(){
        function_id = FunctionId::CALLIBRATION_CMD;
    }
    void setControlData(uint16_t pan_mapped, uint16_t tilt_mapped){
        function_id = FunctionId::ANGLE_PKT;
        data.angle.pan_position = pan_mapped;
        data.angle.tilt_position = tilt_mapped;
    }
    void setTrigger(){
        function_id = FunctionId::TRIGGER_PKT;
    }
    void setSpeedData(double pan_speed,double tilt_speed){
        function_id = FunctionId::SPEED_MODE_PKT;
        data.speed.yaw = abs(pan_speed)*4000;
        data.speed.pitch = abs(tilt_speed)*4000;
        data.speed.yaw_dir = ( (pan_speed > 0) ? 1 : 0 );
        data.speed.pitch_dir = ( (tilt_speed > 0) ? 1 : 0 );
        printf("speed=P,T: %d,%d::%lf,%lf\n",data.speed.yaw,data.speed.pitch,pan_speed,tilt_speed);
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
        case FunctionId::CALLIBRATION_STATUS:
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
        case FunctionId::CALLIBRATION_CMD:
            {
                break;
            }
        case FunctionId::ANGLE_PKT:
            {
                // data buff : 0x0000    0x0000      0x0000     0x0000
                //            [padding]  [pan value] [padding]  [tilt value ] 
                uint16_t * data_buff = (uint16_t*)frame.data;
                
                // memory clean
                data_buff[2] = data.angle.pan_position;   
                data_buff[0] = data.angle.tilt_position;
                break;
            }
            case FunctionId::SPEED_MODE_PKT:
            {
                int16_t * data_buff = (int16_t*)(frame.data+1);
                
                data_buff[0] = data.speed.yaw;   
                data_buff[2] = data.speed.pitch;
                frame.data[0] = data.speed.yaw_dir;
                frame.data[4] = data.speed.pitch_dir;
                break;
            }
            case FunctionId::TRIGGER_PKT:
            {
                int16_t * data_buff = (int16_t*)(frame.data);
                break;
            }
        default:
            {
                break;
            }
        }
        return frame;
    }
};



RxData_t::FunctionId get_function_id(const canid_t& can_id_data){
    CanIdentifier id = get_can_identifier(can_id_data);
    return static_cast<RxData_t::FunctionId>(id.function_id);
}
RxData_t::FunctionId get_rx_data(const can_frame& can_data, RxData_t::Data& data){
    RxData_t::FunctionId function_id = get_function_id(can_data.can_id);
    printf("FNID : %x\n",function_id);
    switch(function_id){
        case RxData_t::FunctionId::BROADCAST_ACK:
            {
                data.broadcast.status = static_cast<payload_type_t>(can_data.data[0]);
                break;
            }
        case RxData_t::FunctionId::CALLIBRATION_ACK:
            {   
                const uint64_t *can_data_buff = (const uint64_t*)can_data.data;
                if(*can_data_buff == 0){
                    data.callibration.status = false;
                } else {
                    const uint16_t *mapped_pos_buff =  (const uint16_t*)can_data.data;
                    gimbal_pos_mapped_t pan_range  = mapped_pos_buff[1];
                    // memcpy(&pan_range, mapped_pos_buff+1, sizeof(pan_range));
                    gimbal_pos_mapped_t tilt_range = mapped_pos_buff[3];
                    // memcpy(&tilt_range, mapped_pos_buff+3, sizeof(tilt_range));
                    data.callibration.pan_config.range = cmd2angle(pan_range);
                    data.callibration.tilt_config.range = cmd2angle(tilt_range);
                    printf("range=P,T : %u,%u:%f,%f\n",pan_range,tilt_range,data.callibration.pan_config.range,data.callibration.tilt_config.range);
                    gimbal_pos_mapped_t pan_home = mapped_pos_buff[0];
                    gimbal_pos_mapped_t tilt_home = mapped_pos_buff[2];
                    data.callibration.pan_config.home_position = cmd2angle(pan_home);
                    data.callibration.tilt_config.home_position = cmd2angle(tilt_home);
                    printf("home=P,T : %u,%u:%f,%f\n",pan_home,tilt_home,data.callibration.pan_config.home_position,data.callibration.tilt_config.home_position);
                    data.callibration.status = true;
                }
                
                
                break;
            }
        case RxData_t::FunctionId::ALIVE_PKT:
            {
                data.is_alive.status = static_cast<bool>(can_data.data[0]);
                break;
            }
        case RxData_t::FunctionId::FEEBACK_PKT:
            {

                break;
            }
        case RxData_t::FunctionId::WHO_AM_I_ACK:
            {
                data.device_id.device_id = static_cast<device_id_t>(can_data.data[0]);
                printf("device id recv %d\n",data.device_id.device_id);
                // print("d")
                break;
            }
        default:
            return function_id;
    }

    return function_id;
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