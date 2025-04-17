#include "arista_camera_middleman/CanDevice.hpp"

#include <cstring>
#include <iostream>
#include <cstdarg>

#define PRIVILEGED_CMD "sudo"

inline std::string get_cmdline(const std::string &cmd, ...)
{
    va_list args;
    va_start(args, cmd);

    std::string full_cmd = cmd;
    const char *arg;
    while ((arg = va_arg(args, const char *)) != nullptr) {
        full_cmd += " ";
        full_cmd += arg;
    }

    va_end(args);
    return full_cmd;
}

inline std::string get_privileged_cmd(const std::string &cmd)
{
    return get_cmdline(PRIVILEGED_CMD, cmd.c_str(), nullptr);
}

inline int exec_cmd(const std::string &cmd)
{
    int ret = system(cmd.c_str());
    printf("running cmd: %s\n", cmd.c_str());
    if (ret == -1) {
        perror("system() failed");
        return -1;
    }
    return ret;
}

inline int exec_privileged(const std::string &cmd)
{
    return exec_cmd(get_privileged_cmd(cmd));
}

namespace arista_camera_middleman
{
CanDevice::CanDevice(const char *can_device_name, int can_bitrate)
{
    can_device_name_ = can_device_name;
    can_bitrate_ = can_bitrate;
    std::cout << "CanDevice::CanDevice() can_device_name: " << can_device_name_ << std::endl;
    std::cout << "CanDevice::CanDevice() can_bitrate: " << can_bitrate_ << std::endl;
    if(this->init()) {
        std::cout << "CanDevice::CanDevice() init success" << std::endl;
    } else {
        std::cout << "CanDevice::CanDevice() init failed" << std::endl;
    }

    
    strcpy(ifr.ifr_name, can_device_name);
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        perror("socket PF_CAN failed");
        exit(EXIT_FAILURE);
    }
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        exit(EXIT_FAILURE);
    }
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}

CanDevice::~CanDevice()
{
    close(socket_);
    set_can_iface(false);
}

bool CanDevice::set_can_iface(bool value)
{
    const char * can_iface_mode = "down";
    if (value) {
        can_iface_mode = "up";
    } 
    std::string cmd = get_privileged_cmd(get_cmdline("ifconfig", can_device_name_.c_str(), can_iface_mode, nullptr));
    int ret = exec_cmd(cmd.c_str());
    if (ret == -1) {
        perror("system() failed");
        return false;
    }
    std::cout << "CanDevice::set_can_iface() " << can_device_name_ << " " << can_iface_mode << std::endl;
    return true;
}

bool CanDevice::init()
{
    set_bitrate(can_bitrate_);
    return true;
}

bool CanDevice::send_can_frame(can_frame *frame)
{
    printf("CanDevice::send_can_frame() frame: %x %lx \n", frame->can_id, (*(long int*)frame->data));
    int nbytes = write(socket_, frame, sizeof(*frame));
    if (nbytes != sizeof(*frame)) {
        perror("Send Error frame[0]!");
        return false;
    }
    return true;
}

int CanDevice::receive_can_frame(can_frame *frame)
{
    int nbytes(0);
    while (true) {
        nbytes = read(socket_, frame, sizeof(*frame));
        if (nbytes < 0) {
            break;
        }
        printf("CanDevice::receive_can_frame() frame: %x %lx \n", frame->can_id, (*(long int*)frame->data));
        if (frame->can_id & CAN_EFF_FLAG) {
            
            return nbytes;
        }
    }
        perror("Receive Error frame[0]!");
        return -1;
}

bool CanDevice::set_filter(std::vector<uint32_t> can_ids)
{
    if (can_ids.empty()) {
        // return clear_filter();
        return true;
    }
    
    struct can_filter *rfilter = new struct can_filter[can_ids.size()];
    
    for (size_t i = 0; i < can_ids.size(); i++) {
        rfilter[i].can_id = can_ids[i];
        rfilter[i].can_mask = CAN_SFF_MASK;
    }
    
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, 
                   sizeof(struct can_filter) * can_ids.size()) < 0) {
        perror("setsockopt failed");
        delete[] rfilter;
        return false;
    }
    
    delete[] rfilter;
    return true;
}

// bool CanDevice::clear_filter()
// {
//     if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) < 0) {
//         perror("setsockopt failed");
//         return false;
//     }
//     return true;
// }

bool CanDevice::set_bitrate(uint32_t can_bitrate)
{
    if (!set_can_iface(false)) {
        std::cerr << "Failed to set CAN interface down" << std::endl;
        return false;
    }
    
    std::string command = get_privileged_cmd(get_cmdline("ip", "link", "set", 
        can_device_name_.c_str(), "type", "can", "bitrate", 
        std::to_string(can_bitrate).c_str(), nullptr));
    
    if (exec_cmd(command) == -1) {
        std::cerr << "Failed to set CAN bitrate" << std::endl;
        return false;
    }
    
    if (!set_can_iface(true)) {
        std::cerr << "Failed to set CAN interface up" << std::endl;
        return false;
    }
    
    std::cout << "CanDevice::set_bitrate() " << can_device_name_ << " " << can_bitrate << std::endl;
    return true;
}

} // namespace arista_camera_middleman 