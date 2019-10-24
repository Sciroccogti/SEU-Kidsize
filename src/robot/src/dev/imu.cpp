#include "imu.hpp"
#include <seumath/math.hpp>
#include <ros/ros.h>

using namespace std;
using namespace seumath;
using namespace Eigen;
using namespace common;

const std::string imu_dev_name = "/dev/ttyTHS2";
const uint32_t imu_baudrate = 115200;

enum 
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

Imu::Imu(): pitch_range_(-30.0, 40.0), roll_range_(-40.0, 40.0)
{
    fall_direction_ = ImuData::FALL_NONE;
    init_dir_ = 0.0;
    is_alive_ = false;
    status = kStatus_Idle;
}

void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}

enum ItemID_t
{
    kItemKeyStatus =            0x80,   /* key status           size: 4 */
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemUID =                  0x91,   /* Unique ID            size: 4 */
    kItemIPAdress =             0x92,   /* ip address           size: 4 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemAccCalibrated =        0xA1,
    kItemAccFiltered =          0xA2,   
    kItemAccLinear =            0xA5,
    kItemGyoRaw =               0xB0,   /* raw gyro             size: 3x2 */  
    kItemGyoCalibrated =        0xB1,
    kItemGyoFiltered =          0xB2, 
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemMagCalibrated =        0xC1,
    kItemMagFiltered =          0xC2,
    kItemRotationEular =        0xD0,   /* eular angle          size:3x2 */
    kItemRotationEular2 =       0xD9,   /* new eular angle      size:3x4 */
    kItemRotationQuat =         0xD1,   /* att q,               size:4x4 */
    kItemTemperature =          0xE0,   
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0x00,   
};

bool Imu::Packet_Decode(uint8_t c)
{
    switch(status)
    {
        case kStatus_Idle:
            if(c == 0x5A)
                status = kStatus_Cmd;
            break;
        case kStatus_Cmd:
            rx_pkt_.type = c;
            switch(rx_pkt_.type)
            {
                case 0xA5:  /* Data */
                    status = kStatus_LenLow;
                    break;
                case 0xA6:  /* Ping */
                    OnDataReceived(rx_pkt_);
                    status = kStatus_Idle;
                    break;
                case 0xA7:  /* Ping Respond */
                    rx_pkt_.ofs = 0;
                    status = kStatus_Data;
                    break;
            }
            break;
        case kStatus_LenLow:
            rx_pkt_.payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            rx_pkt_.payload_len |= (c<<8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;
        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            CRCReceived |= (c<<8);
            rx_pkt_.ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;
        case kStatus_Data:
            rx_pkt_.buf[rx_pkt_.ofs++] = c;
            if(rx_pkt_.type == 0xA7 && rx_pkt_.ofs >= 8)
            {
                rx_pkt_.payload_len = 8;
                OnDataReceived(rx_pkt_);
                status = kStatus_Idle;
            }
            if(rx_pkt_.ofs >= MAX_PACKET_LEN)
            {
                status = kStatus_Idle;
                return false;   
            }

            if(rx_pkt_.ofs >= rx_pkt_.payload_len && rx_pkt_.type == 0xA5)
            {
                /* calculate CRC */
                crc16_update(&CRCCalculated, crc_header, 4);
                crc16_update(&CRCCalculated, rx_pkt_.buf, rx_pkt_.ofs);
                
                /* CRC match */
                if(CRCCalculated == CRCReceived)
                {
                    OnDataReceived(rx_pkt_);
                }
                status = kStatus_Idle;
            }
            break;
        default:
            status = kStatus_Idle;
            break;
    }
    return true;
}

void Imu::OnDataReceived(Packet_t &pkt)
{
	if(pkt.type != 0xA5)
    {
        return;
    }
    int offset = 0;
    uint8_t *p = pkt.buf;
    while(offset < pkt.payload_len)
    {
        switch(p[offset])
        {
            case kItemID:
                id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc, p + offset + 1, sizeof(acc));
                offset += 7;
                break;
            case kItemGyoRaw:
            case kItemGyoCalibrated:
            case kItemGyoFiltered:
                memcpy(gyo, p + offset + 1, sizeof(gyo));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(mag, p + offset + 1, sizeof(mag));
                offset += 7;
                break;
            case kItemRotationEular:
                eular[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
                eular[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
                eular[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
                imu_data_.pitch = eular[0];
                imu_data_.roll = eular[1];
                imu_data_.yaw = eular[2];
                
                if(imu_data_.pitch<pitch_range_.x()) fall_direction_ = ImuData::FALL_BACKWARD;
                else if(imu_data_.pitch>pitch_range_.y()) fall_direction_ = ImuData::FALL_FORWARD;
                else if(imu_data_.roll<roll_range_.x()) fall_direction_ = ImuData::FALL_RIGHT;
                else if(imu_data_.roll>roll_range_.y()) fall_direction_ = ImuData::FALL_LEFT;
                else fall_direction_ = ImuData::FALL_NONE;

                offset += 7;
                break;
            case kItemRotationEular2:
                memcpy(eular, p + offset + 1, sizeof(eular));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat, p + offset + 1, sizeof(quat));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
				return;
                break;
        }
    }

    imu_data_.header.stamp = ros::Time::now();
    imu_data_.yaw = seumath::normalizeDeg<float>(imu_data_.yaw - init_dir_);
}

bool Imu::start()
{
    serial::Timeout to = serial::Timeout::simpleTimeout(5);
    serial_.setPort(imu_dev_name);
    serial_.setBaudrate(imu_baudrate);
    serial_.setTimeout(to);
    try
    {
        serial_.open();
    }
    catch(serial::IOException &e)
    {
        ROS_ERROR("open serial port failed: %s", e.what());
        return false;
    }
    if(!serial_.isOpen())
    {
        return false;
    }

    is_alive_ = true;
    td_ = std::move(thread([this]()
    {
        while(is_alive_ && ros::ok())
        {
            size_t n = serial_.available();
            if(n!=0)
            {
                n = serial_.read(buff_, MAX_PACKET_LEN);
                for(size_t i=0; i<n; i++)
                {
                    Packet_Decode(buff_[i]);
                }
            }
        }
    }));
    return true;
}

void Imu::stop()
{
    serial_.close();
    is_alive_ = false;
}

Imu::~Imu()
{
    if (td_.joinable())
    {
        td_.join();
    }
}
