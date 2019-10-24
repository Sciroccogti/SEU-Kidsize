#ifndef __DEV_COMMON_HPP
#define __DEV_COMMON_HPP

#include <cstdint>

extern void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);

#endif
