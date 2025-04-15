#include "arista_camera_middleman/CamFocuser.hpp"

int main() {
    Focuser focuser(1);
    focuser.reset(Focuser::OPT_FOCUS);
    while (focuser.get(Focuser::OPT_FOCUS) < 18000) {
        focuser.set(Focuser::OPT_FOCUS, focuser.get(Focuser::OPT_FOCUS) + 50);
    }
    focuser.set(Focuser::OPT_FOCUS, 0);
    focuser.set(Focuser::OPT_FOCUS, 10000);
    return 0;
}
