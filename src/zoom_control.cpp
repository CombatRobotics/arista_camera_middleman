//ZOOM PRESETS 

// WIDEST : z: 20000 f: 1000

// Furthest : z : 0  f : 17700
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "arista_camera_middleman/CamFocuser.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

#define RPI_BUS_ID 1

struct ZoomPreset{
    uint16_t zoom;
    uint16_t focus;
};



const ZoomPreset presets[] = {
    {20000, 1000},
    {0, 17700},
};
static constexpr size_t NUM_PRESETS = (sizeof(presets) / sizeof(presets[0]));

class ZoomPresetSubscriber : public rclcpp::Node
{
public:
  ZoomPresetSubscriber() : Node("zoom_presets_subscriber"),focuser(RPI_BUS_ID)
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "preset_idx", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) { this->zoomPresetCallback(msg); });
  };


  inline void set_preset(uint32_t preset_index){
    preset_index = preset_index % NUM_PRESETS;
    const ZoomPreset& preset = presets[preset_index];
    focuser.set(Focuser::OPT_ZOOM, preset.zoom);
    focuser.set(Focuser::OPT_FOCUS, preset.focus);
  };

private:
void zoomPresetCallback(const std_msgs::msg::UInt16::SharedPtr msg){
    set_preset(msg->data);
};
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  Focuser focuser;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZoomPresetSubscriber>();
  node->set_preset(0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
