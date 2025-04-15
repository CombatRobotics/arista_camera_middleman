#ifndef FOCUSER_H
#define FOCUSER_H

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <vector>

class Focuser {
public:
    static constexpr uint8_t CHIP_I2C_ADDR = 0x0C;
    static constexpr uint8_t BUSY_REG_ADDR = 0x04;

    enum Option {
        OPT_FOCUS   = 0x1001,
        OPT_ZOOM    = 0x1002,
        OPT_MOTOR_X = 0x1003,
        OPT_MOTOR_Y = 0x1004,
        OPT_IRCUT   = 0x1005
    };

    struct OptionInfo {
        uint8_t regAddr;
        uint16_t minValue;
        uint16_t maxValue;
        int resetAddr;  // use -1 to indicate no reset register
    };

private:
    int file;
    std::string devicePath;
    std::map<Option, OptionInfo> opts;

public:
    std::vector<uint16_t> starting_point = {
        10000, 10000, 10000, 10720, 8070, 5970,
        4320, 2920, 1920, 970, 520, 20, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    };

    std::vector<uint16_t> end_point = {
        20000, 20000, 20000, 20000, 19620, 17020,
        14920, 13170, 12020, 10970, 10170, 9770,
        9170, 9020, 8820, 8570, 8570, 8570,
        8770, 8970, 9170
    };

    Focuser(int bus) {
        devicePath = "/dev/i2c-" + std::to_string(bus);
        file = open(devicePath.c_str(), O_RDWR);
        if (file < 0) {
            perror("Unable to open I2C bus");
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, CHIP_I2C_ADDR) < 0) {
            perror("Unable to select I2C device");
            exit(1);
        }

        opts = {
            { OPT_FOCUS,   {0x01, 0, 20000, 0x0B} },
            { OPT_ZOOM,    {0x00, 3000, 20000, 0x0A} },
            { OPT_MOTOR_X, {0x05, 0, 180, -1} },
            { OPT_MOTOR_Y, {0x06, 0, 180, -1} },
            { OPT_IRCUT,   {0x0C, 0x00, 0x01, -1} }
        };
    }

    ~Focuser() {
        if (file >= 0) {
            close(file);
        }
    }

    uint16_t readWord(uint8_t regAddr) {
        uint8_t buf[2];
        if (::write(file, &regAddr, 1) != 1) {
            perror("I2C: Write regAddr failed");
            return 0;
        }
        if (::read(file, buf, 2) != 2) {
            perror("I2C: Read failed");
            return 0;
        }
        return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }

    void writeWord(uint8_t regAddr, uint16_t value) {
        uint8_t buf[3];
        buf[0] = regAddr;
        buf[1] = (value >> 8) & 0xFF; // MSB
        buf[2] = value & 0xFF;        // LSB
        if (::write(file, buf, 3) != 3) {
            perror("I2C: Write failed");
        }
    }

    bool isBusy() {
        return readWord(BUSY_REG_ADDR) != 0;
    }

    void waitingForFree() {
        int count = 0;
        while (isBusy() && count < 500) {
            count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void reset(Option opt, int flag = 1) {
        waitingForFree();
        OptionInfo info = opts[opt];
        if (info.resetAddr != -1) {
            writeWord(info.resetAddr, 0x0000);
            set(opt, info.minValue, flag);
        }
    }

    uint16_t get(Option opt, int flag = 0) {
        waitingForFree();
        OptionInfo info = opts[opt];
        return readWord(info.regAddr);
    }

    void set(Option opt, uint16_t value, int flag = 1) {
        waitingForFree();
        OptionInfo info = opts[opt];
        if (value > info.maxValue) value = info.maxValue;
        if (value < info.minValue) value = info.minValue;
        writeWord(info.regAddr, value);
        if (flag & 0x01) waitingForFree();
    }
};

#endif // FOCUSER_H
