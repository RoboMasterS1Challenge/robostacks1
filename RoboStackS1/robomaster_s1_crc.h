#include <stdio.h>
#include <stdint.h>

#ifndef __ROBOMASTER_S1_CRC_H__
#define __ROBOMASTER_S1_CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

  uint8_t getCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
  uint32_t verifyCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
  void appendCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);
  uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
  uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
  void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif

#endif
