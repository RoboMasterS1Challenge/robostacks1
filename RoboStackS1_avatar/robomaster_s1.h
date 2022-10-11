#include <stdio.h>
#include <stdint.h>
#include "robomaster_s1_crc.h"

#ifndef __ROBOMASTER_S1_H__
#define __ROBOMASTER_S1_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

#define CAN_ID_NUM 9

#define CAN_RX_BUFFER_SIZE 2560
#define RX_BUFFER_SIZE 2048
#define TX_BUFFER_SIZE 2048

  typedef union float_uint8
  {
    float float_data;
    uint8_t uint8_data[4];
  } float_uint8;

  typedef enum can_ids
  {
    // ID_0x201 = 0,
    ID_0x202 = 0,
    ID_0x203 = 1,
    ID_0x204 = 2,
    ID_0x211 = 3,
    ID_0x212 = 4,
    ID_0x213 = 5,
    ID_0x214 = 6,
    ID_0x215 = 7,
    ID_0x216 = 8
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

  typedef struct quaternion
  {
    float x;
    float y;
    float z;
    float w;
  } quaternion;

  int parseCanData(uint16_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);
  quaternion from_euler_to_quaternion(float roll, float pitch, float yaw);
  void quaternion_to_euler(quaternion q, float *roll, float *pitch, float *yaw);
#ifdef __cplusplus
}
#endif

#endif
