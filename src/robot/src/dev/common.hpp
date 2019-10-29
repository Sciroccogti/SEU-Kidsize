#ifndef __DEV_COMMON_HPP
#define __DEV_COMMON_HPP

#include <cstdint>
#define DXL_ZERO_POS 2048
#define DXL_MAX_POS  4096
#define DXL_MIN_POS  0

inline void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
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

inline uint32_t float2dxl(float deg)
{
    return static_cast<uint32_t>(deg / (360.0f / (float)(DXL_MAX_POS - DXL_MIN_POS)) + DXL_ZERO_POS);
}

#endif
