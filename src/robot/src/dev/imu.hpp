#ifndef __IMU_HPP
#define __IMU_HPP

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <common/ImuData.h>
#include <serial/serial.h>
#include "common.hpp"

class Imu
{
public:
    enum {MAX_PACKET_LEN=128};
    struct Packet_t
    {
        uint32_t ofs;
        uint8_t buf[MAX_PACKET_LEN];    /* total frame buffer */
        uint16_t payload_len;           
        uint16_t len;                   /* total frame len */
        uint8_t type;
    };

    Imu();
    ~Imu();

    bool start();
    void stop();
private:
    bool Packet_Decode(uint8_t c);
    void OnDataReceived(Packet_t &pkt);

    unsigned char buff_[2];
    common::ImuData imu_data_;
    std::thread td_;

    serial::Serial serial_;
    std::atomic_int fall_direction_;

    const Eigen::Vector2f pitch_range_;
    const Eigen::Vector2f roll_range_;

    Packet_t rx_pkt_;
    int16_t acc[3];
    int16_t gyo[3];
    int16_t mag[3];
    float eular[3];
    float quat[4];
    uint8_t id;
    uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    uint8_t status;
    uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    bool is_alive_;
    float init_dir_;
};

#endif
