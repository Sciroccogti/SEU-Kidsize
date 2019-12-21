#include "mcu.hpp"
#include <common/common.hpp>

using namespace std;

const string mcu_dev_name = "/dev/ttyUSB0";
const uint32_t mcu_baudrate = 576000;
const uint32_t mcu_timeout = 10;

const uint16_t REQ_PKT_LEN = 16;
const uint16_t LEAST_PKT_LEN = 7;

#define PKT_TYPE    2
#define PKT_LEN_L   3
#define PKT_LEN_H   4
#define PKT_PARAM_START 5

#define REQ_DATA    1
#define ACK_DATA    2
#define MOTOR_DATA  3
#define LED_DATA    4

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
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

bool Mcu::open()
{
    serial_ = std::make_shared<Serial>(mcu_dev_name.c_str());
    if(!serial_->openPort())
    {
        ROS_WARN("Failed to open the port!");
        is_open_ = false;
        return false;
    }
    if(!serial_->setBaudRate(mcu_baudrate))
    if(!serial_->openPort())
    {
        ROS_WARN("Failed to change the baudrate!");
        is_open_ = false;
        return false;
    }
    is_open_ = true;
    return true;
}

void Mcu::close()
{
    if(is_open_)
        serial_->closePort();
}

void Mcu::writePacket(const McuPacket &pkt)
{
    uint8_t send_buff[MAX_MCU_BUFF_LEN] = {0x5A, 0xA5};
    send_buff[PKT_TYPE] = pkt.type;
    send_buff[PKT_LEN_L] = SEU_LOBYTE(pkt.len);
    send_buff[PKT_LEN_H] = SEU_HIBYTE(pkt.len);
    memcpy(send_buff+PKT_PARAM_START, pkt.data, pkt.len);
    uint16_t crc=0;
    crc16_update(&crc, send_buff+PKT_TYPE, pkt.len+3);
    send_buff[PKT_PARAM_START+pkt.len+0] = SEU_LOBYTE(crc);
    send_buff[PKT_PARAM_START+pkt.len+1] = SEU_HIBYTE(crc);
    serial_->writePort(send_buff, pkt.len+LEAST_PKT_LEN);
}

bool Mcu::readPacket(Mcu::McuPacket &pkt)
{
    uint8_t recv_buff[MAX_MCU_BUFF_LEN];    
    uint16_t rx_length     = 0;
    uint16_t wait_length   = LEAST_PKT_LEN;
    serial_->setPacketTimeout((double)(mcu_timeout));
    while(ros::ok())
    {
        rx_length += serial_->readPort(&recv_buff[rx_length], wait_length - rx_length);
        if (rx_length >= wait_length)
        {
            uint16_t idx = 0;

            // find packet header
            for (idx = 0; idx < (rx_length - 1); idx++)
            {
                if ((recv_buff[idx] == 0x5A) && (recv_buff[idx+1] == 0xA5))
                break;
            }

            if (idx == 0)   // found at the beginning of the packet
            {
                // re-calculate the exact length of the rx packet
                if (wait_length != SEU_MAKEWORD(recv_buff[PKT_LEN_L], recv_buff[PKT_LEN_H]) + LEAST_PKT_LEN)
                {
                    wait_length = SEU_MAKEWORD(recv_buff[PKT_LEN_L], recv_buff[PKT_LEN_H]) + LEAST_PKT_LEN;
                    continue;
                }

                if (rx_length < wait_length)
                {
                    // check timeout
                    if (serial_->isPacketTimeout() == true)
                    {
                        return false;
                    }
                    else
                    {
                        continue;
                    }
                }
                uint16_t crc=0;
                crc16_update(&crc, recv_buff+PKT_TYPE, 3+SEU_MAKEWORD(recv_buff[PKT_LEN_L], recv_buff[PKT_LEN_H]));
                if(crc == SEU_MAKEWORD(recv_buff[wait_length-2], recv_buff[wait_length-1]))
                {
                    pkt.type = recv_buff[PKT_TYPE];
                    pkt.len = SEU_MAKEWORD(recv_buff[PKT_LEN_L], recv_buff[PKT_LEN_H]);
                    memcpy(pkt.data, recv_buff+PKT_PARAM_START, pkt.len);
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                // remove unnecessary packets
                for (uint16_t s = 0; s < rx_length - idx; s++)
                    recv_buff[s] = recv_buff[idx + s];
                rx_length -= idx;
            }
        }
        else
        {
            // check timeout
            if (serial_->isPacketTimeout() == true)
            {
                return false;
            }
        }
        usleep(0);
    }
    return false;
}

