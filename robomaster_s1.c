#include <stdio.h>
#include <string.h>
#include <limits.h> // CHAR_BIT
#include <stdint.h>
#include <stdlib.h>

#include "robomaster_s1.h"
#ifdef __cplusplus
extern "C" {
#endif

twist command_twist;
int command_blaster;
int command_gel_blaster;
int command_lose;
led command_led;

volatile uint8_t robomas_buffer[CAN_ID_NUM][BUFFER_SIZE];
volatile int robomas_buffer_rp[CAN_ID_NUM];
volatile int robomas_buffer_wp[CAN_ID_NUM];
volatile uint8_t command_buffer[BUFFER_SIZE];
volatile int command_buffer_rp;
volatile int command_buffer_wp;


int parseCanData(uint16_t id, uint8_t* in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t* out_data_size)
{
  int i;

  for(int i=0;i<in_data_size;i++){
    robomas_buffer[id][robomas_buffer_wp[id]] = in_data[i];
    robomas_buffer_wp[id]++;
    robomas_buffer_wp[id] %= BUFFER_SIZE;
  }

  int can_in_buffer_size = (robomas_buffer_wp[id] - robomas_buffer_rp[id] + BUFFER_SIZE) % BUFFER_SIZE;

  // Data size check
  if(can_in_buffer_size < 7){
    return 0;
  }

  // Search Header
  int find_flag = 0;
  for(i = 0; i < can_in_buffer_size - 7; i++){
    if(robomas_buffer[id][(robomas_buffer_rp[id])%BUFFER_SIZE]==0x55 && robomas_buffer[id][(robomas_buffer_rp[id]+2)%BUFFER_SIZE]==0x04){
      find_flag = 1;
      break;
    }
    can_in_buffer_size--;
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
  }

  if(find_flag == 0){
    return 0;
  }

  // Check data length
  int send_data_size = robomas_buffer[id][(robomas_buffer_rp[id]+1)%BUFFER_SIZE];
  if(send_data_size > can_in_buffer_size)
  {
    // Not enough data
    return 0;
  }

  // Prepare send data
  uint8_t* send_data;
  send_data = (uint8_t*)malloc(sizeof(uint8_t) * send_data_size);


  for(i = 0; i < send_data_size; i++){
    int buffer_p = (robomas_buffer_rp[id] + i) % BUFFER_SIZE;
    send_data[i] = robomas_buffer[id][buffer_p];
  }

  // Check header crc8
  if(!verifyCRC8CheckSum(send_data, 4))
  {
    // checksum error
    // skip header
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  // Check crc16
  if(!verifyCRC16CheckSum(send_data, send_data_size))
  {
    // checksum error
    // skip header
    robomas_buffer_rp[id]++;
    robomas_buffer_rp[id] %= BUFFER_SIZE;
    free(send_data);
    return 0;
  }

  memcpy(out_data, send_data, send_data_size);
  *out_data_size = send_data_size;

  free(send_data);
  robomas_buffer_rp[id] += send_data_size;
  robomas_buffer_rp[id] %= BUFFER_SIZE;
  return 1;
}


#ifdef __cplusplus
}
#endif