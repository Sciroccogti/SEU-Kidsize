#ifndef __SEU_COMMON_HPP
#define __SEU_COMMON_HPP

#include <stdint.h>

#define SEU_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define SEU_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define SEU_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define SEU_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define SEU_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define SEU_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#endif