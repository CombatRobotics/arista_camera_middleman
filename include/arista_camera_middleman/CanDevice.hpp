#pragma once

#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <string>
namespace arista_camera_middleman
{
struct CanDevice
{
public:
    CanDevice(const char *can_device_name, int can_bitrate);
    ~CanDevice();
    bool init();
    bool send_can_frame(struct can_frame *frame);
    int receive_can_frame(struct can_frame *frame);
    int recv_can_timeout(can_frame *frame, uint32_t timeout_ms);
    bool set_filter(std::vector<uint32_t> can_ids);
    // bool clear_filter();
    bool set_bitrate(uint32_t can_bitrate);
    bool set_can_iface(bool value);
    inline int get_socket_fd() const { return socket_; }
private:
    std::string can_device_name_;
    int can_bitrate_;
    int socket_;
    struct sockaddr_can addr;
    struct ifreq ifr;
};
} // namespace arista_camera_middleman
