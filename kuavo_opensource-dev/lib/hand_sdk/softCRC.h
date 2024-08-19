
/* specifically for storing char data in embedded system */

#ifndef _SOFTCRC_H
#define _SOFTCRC_H

#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C"
{
#endif
    uint32_t softCRC_CRC32(const void *pData, size_t Length, uint32_t seed, uint32_t final);
    uint16_t softCRC_CRC16_CCITT(const void *pData, size_t length, uint16_t seed, uint16_t final);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* _SOFTCRC_H */
