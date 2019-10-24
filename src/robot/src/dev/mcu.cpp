#include "mcu.hpp"
#include <common/common.hpp>
#include "common.hpp"

using namespace std;

const string mcu_dev_name = "/dev/ttyUSB0";
const uint32_t mcu_baudrate = 230400;
const uint32_t mcu_timeout = 25;
#define PKT_TYPE    2
#define PKT_LEN_L   3
#define PKT_LEN_H   4
#define PKT_PARAM_START 5

#define REQ_DATA    1
#define ACK_DATA    2
#define MOTOR_DATA  3
#define LED_DATA    4

enum 
{
    Status_5A,
    Status_A5,
    Status_Type,
    Status_LenL,
    Status_LenH,
    Status_Data
};

bool Mcu::open()
{
    serial::Timeout to = serial::Timeout::simpleTimeout(mcu_timeout);
    serial_.setPort(mcu_dev_name);
    serial_.setBaudrate(mcu_baudrate);
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
    return true;
}

Mcu::McuPacket Mcu::readPacket()
{
    uint8_t recv_buff[MAX_MCU_BUFF_LEN];    
    uint32_t recv_len=0;
    uint8_t led=1;
    uint8_t type=0;
    uint16_t pkt_len = 0;
    uint32_t pcnt = 0;
    uint8_t recv_status = Status_5A;
    Mcu::McuPacket pkt;
    while(ros::ok())
    {
        size_t n = 0;
        n = serial_.read(buffer, n);
        if(n == 0){
            ROS_WARN("mcu serial timeout");
            break;
        }
        for(int i=0; i<n; i++)
        {
            uint8_t c=buffer[i];
            switch(recv_status)
            {
            case Status_5A:
                if(c==0x5A)
                {
                    recv_status = Status_A5;
                    recv_buff[recv_len++] = c;
                }
                break;
            case Status_A5:
                if(c==0xA5)
                {
                    recv_status = Status_Type;
                    recv_buff[recv_len++] = c;
                }
                else
                {
                    recv_status = Status_5A;
                    recv_len = 0;
                }
                break;
            case Status_Type:
                recv_buff[recv_len++] = c;
                recv_status = Status_LenL;
                type = c;
                break;
            case Status_LenL:
                recv_buff[recv_len++] = c;
                recv_status = Status_LenH;
                break;
            case Status_LenH:
                recv_buff[recv_len++] = c;
                pkt_len = SEU_MAKEWORD(recv_buff[recv_len-2], c);
                recv_status = Status_Data;
                break;
            case Status_Data:
                recv_buff[recv_len++] = c;
                break;
            default:
                break;
            }
            if(recv_len == pkt_len + PKT_PARAM_START + 2)
            {
                uint16_t crc=0;
                crc16_update(&crc, recv_buff+PKT_TYPE, 3+pkt_len);
                if(crc == SEU_MAKEWORD(recv_buff[recv_len-2], recv_buff[recv_len-1]))
                {
                    pkt.type = type;
                    pkt.len = pkt_len;
                    memcpy(pkt.data, recv_buff+PKT_PARAM_START, pkt_len);
                    return pkt;
                }
                bzero(recv_buff, MAX_MCU_BUFF_LEN);
                recv_len = 0;
                pkt_len = 0;
                recv_status = Status_5A;
            }
        }
    }
    return pkt;
}

