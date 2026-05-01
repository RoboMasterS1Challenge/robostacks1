#include <stdio.h>
#include <string.h>
#include <limits.h> // CHAR_BIT
#include <stdint.h>
#include <math.h>

#include "robomaster_s1.h"
#ifdef __cplusplus
extern "C"
{
#endif

  uint8_t robomas_buffer[CAN_ID_NUM][RX_BUFFER_SIZE];
  uint16_t robomas_buffer_rp[CAN_ID_NUM];
  uint16_t robomas_buffer_wp[CAN_ID_NUM];

  static uint16_t ring_available(uint16_t read_pos, uint16_t write_pos)
  {
    return (uint16_t)((write_pos - read_pos + RX_BUFFER_SIZE) % RX_BUFFER_SIZE);
  }

  static void ring_drop_byte(uint16_t id)
  {
    robomas_buffer_rp[id] = (uint16_t)((robomas_buffer_rp[id] + 1) % RX_BUFFER_SIZE);
  }

  static float safe_asin(float value)
  {
    if (value >= 1.0f)
    {
      return (float)M_PI / 2.0f;
    }
    if (value <= -1.0f)
    {
      return (float)-M_PI / 2.0f;
    }
    return asinf(value);
  }

  int parseCanData(uint16_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size)
  {
    int i;

    if (id >= CAN_ID_NUM || in_data == NULL || out_data == NULL || out_data_size == NULL)
    {
      return 0;
    }

    for (int i = 0; i < in_data_size; i++)
    {
      robomas_buffer[id][robomas_buffer_wp[id]] = in_data[i];
      robomas_buffer_wp[id] = (uint16_t)((robomas_buffer_wp[id] + 1) % RX_BUFFER_SIZE);

      if (robomas_buffer_wp[id] == robomas_buffer_rp[id])
      {
        ring_drop_byte(id);
      }
    }

    uint16_t can_in_buffer_size = ring_available(robomas_buffer_rp[id], robomas_buffer_wp[id]);

    // Data size check
    if (can_in_buffer_size < 7)
    {
      return 0;
    }

    // Search Header
    int find_flag = 0;
    for (i = 0; i < can_in_buffer_size - 7; i++)
    {
      if (robomas_buffer[id][(robomas_buffer_rp[id]) % RX_BUFFER_SIZE] == 0x55 && robomas_buffer[id][(robomas_buffer_rp[id] + 2) % RX_BUFFER_SIZE] == 0x04)
      {
        find_flag = 1;
        break;
      }
      can_in_buffer_size--;
      ring_drop_byte(id);
    }

    if (find_flag == 0)
    {
      return 0;
    }

    // Check data length
    uint8_t send_data_size = robomas_buffer[id][(robomas_buffer_rp[id] + 1) % RX_BUFFER_SIZE];
    if (send_data_size < 7)
    {
      ring_drop_byte(id);
      return 0;
    }

    if (send_data_size > can_in_buffer_size)
    {
      // Not enough data
      return 0;
    }

    uint8_t send_data[0xFF];

    for (i = 0; i < send_data_size; i++)
    {
      int buffer_p = (robomas_buffer_rp[id] + i) % RX_BUFFER_SIZE;
      send_data[i] = robomas_buffer[id][buffer_p];
    }

    // Check header crc8
    if (!verifyCRC8CheckSum(send_data, 4))
    {
      // checksum error
      // skip header
      ring_drop_byte(id);
      return 0;
    }

    // Check crc16
    if (!verifyCRC16CheckSum(send_data, send_data_size))
    {
      // checksum error
      // skip header
      ring_drop_byte(id);
      return 0;
    }

    memcpy(out_data, send_data, send_data_size);
    *out_data_size = send_data_size;

    robomas_buffer_rp[id] = (uint16_t)((robomas_buffer_rp[id] + send_data_size) % RX_BUFFER_SIZE);
    return 1;
  }

  quaternion from_euler_to_quaternion(float roll, float pitch, float yaw)
  {
    struct quaternion q;
    float cr2 = cosf(roll * 0.5f);
    float cp2 = cosf(pitch * 0.5f);
    float cy2 = cosf(yaw * 0.5f);
    float sr2 = sinf(roll * 0.5f);
    float sp2 = sinf(pitch * 0.5f);
    float sy2 = sinf(yaw * 0.5f);

    q.w = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q.x = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q.y = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q.z = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
    return q;
  }

  void quaternion_to_euler(quaternion q, float *roll, float *pitch, float *yaw)
  {
    float q0q0 = q.w * q.w;
    float q1q1 = q.x * q.x;
    float q2q2 = q.y * q.y;
    float q3q3 = q.z * q.z;
    float q0q1 = q.w * q.x;
    float q0q2 = q.w * q.y;
    float q0q3 = q.w * q.z;
    float q1q2 = q.x * q.y;
    float q1q3 = q.x * q.z;
    float q2q3 = q.y * q.z;

    if (roll)
    {
      *roll = atan2f((2.f * (q2q3 + q0q1)), (q0q0 - q1q1 - q2q2 + q3q3));
    }
    if (pitch)
    {
      *pitch = -safe_asin((2.f * (q1q3 - q0q2)));
    }
    if (yaw)
    {
      *yaw = atan2f((2.f * (q1q2 + q0q3)), (q0q0 + q1q1 - q2q2 - q3q3));
    }
  }
#ifdef __cplusplus
}
#endif
