#ifndef __MCU_HPP
#define __MCU_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include "common.hpp"

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
    struct McuPacket
    {
        uint8_t type=0;
        uint16_t len=0;
        uint8_t data[MAX_MCU_BUFF_LEN];
    };

    McuPacket readRequest();
    void sendPacket(const McuPacket &pkt);

    bool open();
    void close();
    bool isOpen()
    {
        return serial_.isOpen();
    }
private:
    serial::Serial serial_;
    uint8_t buffer[MAX_MCU_BUFF_LEN];
};

#endif
