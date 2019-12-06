#ifndef __MCU_HPP
#define __MCU_HPP

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <memory>
#include "common.hpp"
#include "serial.h"

#define MAX_MCU_BUFF_LEN 256

class Mcu
{
public:
    enum
    {
        REQ_DATA = 1,
        ACK_DATA = 2,
        MOTOR_DATA = 3,
        LED_DATA = 4
    };
    enum
    {
        REQ_DATA_IMU_OFFSET = 3
    };
    struct McuPacket
    {
        uint8_t type=0;
        uint16_t len=0;
        uint8_t data[MAX_MCU_BUFF_LEN];
    };

    bool readPacket(McuPacket &pkt);
    void writePacket(const McuPacket &pkt);

    bool open();
    void close();
    bool isOpen()
    {
        return is_open_;
    }
private:
    std::shared_ptr<Serial> serial_;
    bool is_open_;
    uint8_t buffer[MAX_MCU_BUFF_LEN];
};

#endif
