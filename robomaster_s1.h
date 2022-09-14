#include <stdio.h>
#include <stdint.h>
#include "robomaster_s1_crc.h"

#ifndef __ROBOMASTER_S1_H__
#define __ROBOMASTER_S1_H__


#ifdef __cplusplus
extern "C" {
#endif

#define CAN_ID_NUM 10

#define BUFFER_SIZE 2048

#define TWIST_COMMAND_SIZE 19
#define BLASTER_COMMAND_SIZE 8
#define LOSE_COMMAND_SIZE 8
#define LED_COMMAND_SIZE 10

#define COMMAND_LIST_SIZE 39

typedef union float_uint8 {
  float float_data;
  uint8_t uint8_data[4];
} float_uint8;

typedef enum can_ids
{
    ID_0x201 = 0,
    ID_0x202 = 1,
    ID_0x203 = 2,
    ID_0x204 = 3,
    ID_0x211 = 4,
    ID_0x212 = 5,
    ID_0x213 = 6,
    ID_0x214 = 7,
    ID_0x215 = 8,
    ID_0x216 = 9
} can_ids;

typedef struct CANRxMsg
{
    can_ids can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANRxMsg;

typedef struct linear
{
    double x;
    double y;
    double z;
} linear;
typedef struct angular
{
    double x;
    double y;
    double z;
} angular;
typedef struct twist
{
    uint8_t enable;
    linear twist_linear;
    angular twist_angular;
} twist;

typedef struct led
{
    uint8_t enable;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led;

int parseCanData(uint16_t id, uint8_t* in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t* out_data_size);


#ifdef __cplusplus
}
#endif

#endif
