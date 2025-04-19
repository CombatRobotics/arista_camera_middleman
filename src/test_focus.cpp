#include "arista_camera_middleman/CamFocuser.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

// Set terminal to raw mode to capture key presses immediately
void setRawMode(bool enable) {
    static struct termios oldt;
    struct termios newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // Disable buffering and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal
    }
}

// Non-blocking keyboard check
bool kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, nullptr, nullptr, &tv);
}

int main() {
    int bus;
    std::cout << "Enter I2C Bus Number (e.g. 1 for /dev/i2c-1): ";
    std::cin >> bus;

    Focuser focuser(bus);

    uint16_t focus = focuser.get(Focuser::OPT_FOCUS);
    uint16_t zoom = focuser.get(Focuser::OPT_ZOOM);
    uint16_t motorX = focuser.get(Focuser::OPT_MOTOR_X);
    uint16_t motorY = focuser.get(Focuser::OPT_MOTOR_Y);
    uint16_t ircut = focuser.get(Focuser::OPT_IRCUT);

    const uint16_t step = 100;
    char ch;
    setRawMode(true);

    std::cout << "\nUse keys to control:\n"
              << "Zoom (a/d), Focus (w/s), Motor X (j/l), Motor Y (i/k), IRCut (o/p), Quit (q)\n";

    while (true) {
        if (kbhit()) {
            ch = getchar();

            switch (ch) {
                case 'w':
                    focus += step;
                    focuser.set(Focuser::OPT_FOCUS, focus);
                    std::cout << "\nFocus: " << focus;
                    break;
                case 's':
                    focus -= step;
                    focuser.set(Focuser::OPT_FOCUS, focus);
                    std::cout << "\nFocus: " << focus;
                    break;
                case 'a':
                    zoom -= step;
                    focuser.set(Focuser::OPT_ZOOM, zoom);
                    std::cout << "\nZoom: " << zoom;
                    break;
                case 'd':
                    zoom += step;
                    focuser.set(Focuser::OPT_ZOOM, zoom);
                    std::cout << "\nZoom: " << zoom;
                    break;
                case 'j':
                    motorX -= 1;
                    focuser.set(Focuser::OPT_MOTOR_X, motorX);
                    std::cout << "\nMotor X: " << motorX;
                    break;
                case 'l':
                    motorX += 1;
                    focuser.set(Focuser::OPT_MOTOR_X, motorX);
                    std::cout << "\nMotor X: " << motorX;
                    break;
                case 'i':
                    motorY += 1;
                    focuser.set(Focuser::OPT_MOTOR_Y, motorY);
                    std::cout << "\nMotor Y: " << motorY;
                    break;
                case 'k':
                    motorY -= 1;
                    focuser.set(Focuser::OPT_MOTOR_Y, motorY);
                    std::cout << "\nMotor Y: " << motorY;
                    break;
                case 'o':
                    ircut = 0;
                    focuser.set(Focuser::OPT_IRCUT, ircut);
                    std::cout << "\nIRCut: OFF";
                    break;
                case 'p':
                    ircut = 1;
                    focuser.set(Focuser::OPT_IRCUT, ircut);
                    std::cout << "\nIRCut: ON";
                    break;
                case 'q':
                    std::cout << "\nExiting...\n";
                    setRawMode(false);
                    return 0;
                default:
                    break;
            }

            std::cout.flush();
        }

        usleep(5000); // Small delay to avoid high CPU usage
    }

    setRawMode(false);
    return 0;
}
