
#include <M5Core2.h>
#include <PS4Controller.h>
#include <CAN.h>

#include "robomaster_s1.h"
#include "command_list.h"

hw_timer_t *timer = NULL;

uint16_t command_counter[COMMAND_LIST_SIZE];
uint16_t counter_led = 0;
uint16_t counter_blaster = 0;
uint16_t counter_gel_blaster = 0;
uint16_t counter_lose = 0;
uint16_t counter_mode = 0;

// Command
twist command_vel;
int command_blaster;
int command_gel_blaster;
int command_lose;
int command_control_mode;
led command_led;

#define COMMAND_VEL_TIMEOUT 150
uint16_t command_vel_timeout_count = COMMAND_VEL_TIMEOUT;

#define ROBOMASTER_S1_CAN_TIMEOUT 100
uint16_t robomasters1_can_timeout_count = ROBOMASTER_S1_CAN_TIMEOUT;
uint8_t robomasters1_can_timeout = 0;

CANRxMsg rx_msg_buffer[CAN_RX_BUFFER_SIZE];
int buffer_rp = 0;
int buffer_wp = 0;

uint8_t can_command_buffer[TX_BUFFER_SIZE];
int can_command_buffer_rp = 0;
int can_command_buffer_wp = 0;

int initial_task_number = 0;

uint8_t received_data[0xFF];
uint8_t received_data_size = 0;

float gimbal_base_pan_angle = 0;
float gimbal_map_pan_angle = 0;
float gimbal_base_tilt_angle = 0;
float gimbal_map_tilt_angle = 0;

// Timer
uint8_t timer10sec_flag = 0;
uint32_t timer10sec_counter = 0;
uint8_t timer1sec_flag = 0;
uint32_t timer1sec_counter = 0;
uint8_t timer10msec_flag = 0;
uint64_t timer10msec_counter = 0;
uint8_t timer100msec_flag = 0;
uint32_t timer100msec_counter = 0;

// LED
#define LED_LIST_NUM 4
led led_white = {1, 255, 255, 255};
led led_red = {1, 255, 0, 0};
led led_green = {1, 0, 255, 0};
led led_blue = {1, 0, 0, 255};
led led_list[LED_LIST_NUM] = {led_white, led_red, led_green, led_blue};
int led_list_count = 0;

// Wheel
uint32_t odom_counter;
uint32_t odom_counter2;
float wheel_angle[4];
int16_t wheel_angular_velocity[4];
uint32_t m_bus_update_count[4];

// IMU
float mag_x;
float mag_y;
float g_x;
float g_y;
float g_z;
float gyro_x;
float gyro_y;
float gyro_z;
float roll, pitch, yaw;

// chassis
float filtered_vel_x1;
float filtered_vel_y1;
float filtered_vel_x2;
float filtered_vel_y2;
float unknown_data;
quaternion base_pose_quaternion;
float base_odom_yaw;
int16_t voltage;
int16_t temperature;
uint32_t current;
uint8_t battery_soc;

// multi thread
TaskHandle_t thp[1];

// Multi thread core0 task
void Core0a(void *args)
{
  while(1)
  {
    delayMicroseconds(10);
  }
}

void IRAM_ATTR on10msecTimer()
{
  // 10msec Timer
  timer10msec_counter++;
  timer10msec_flag = 1;

  // 10sec conter
  if (timer10msec_counter % 1000 == 0)
  {
    timer10sec_flag = 1;
  }

  // 1sec conter
  if (timer10msec_counter % 100 == 0)
  {
    timer1sec_flag = 1;
  }

  // 100msec counter
  if (timer10msec_counter % 10 == 0)
  {
    timer100msec_flag = 1;
  }

  command_vel_timeout_count--;

  if (command_vel_timeout_count == 0)
  {
    command_vel.enable = false;
    command_vel_timeout_count = 1;
  }

  robomasters1_can_timeout_count--;
  if (robomasters1_can_timeout_count == 0)
  {
    robomasters1_can_timeout_count = 1;
    robomasters1_can_timeout = 1;
  }
}

void initializeTimer(void)
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &on10msecTimer, true);
  timerAlarmWrite(timer, 10000, true); // 10msec
  timerAlarmEnable(timer);
}

void set_can_command(uint8_t command_no)
{
  uint8_t command_length = can_command_list[command_no][3];
  uint8_t header_command[0xFF];
  uint8_t idx = 0;
  for (int i = 2; i < command_length; i++)
  {
    header_command[idx] = can_command_list[command_no][i];

    if (i == 5 && can_command_list[command_no][5] == 0xFF)
    {
      appendCRC8CheckSum(header_command, 4);
    }
    if (i == 8 && can_command_list[command_no][8] == 0xFF)
    {
      header_command[idx] = command_counter[command_no] & 0xFF;
    }
    if (i == 9 && can_command_list[command_no][9] == 0xFF)
    {
      header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
    }
    command_counter[command_no]++;

    idx++;
  }
  appendCRC16CheckSum(header_command, command_length);
  for (int i = 0; i < command_length; i++)
  {
    can_command_buffer[can_command_buffer_wp] = header_command[i];
    can_command_buffer_wp++;
    can_command_buffer_wp %= TX_BUFFER_SIZE;
  }
}

void onReceive(int packetSize)
{

  if (CAN.packetExtended())
  {
    return;
  }

  if (CAN.packetRtr())
  {
    return;
  }

  CANRxMsg msg;
  switch (CAN.packetId())
  {
  case 0x202:
    msg.can_id = ID_0x202;
    break;
  case 0x203:
    msg.can_id = ID_0x203;
    break;
  case 0x204:
    msg.can_id = ID_0x204;
    break;
  case 0x211:
    msg.can_id = ID_0x211;
    break;
  case 0x212:
    msg.can_id = ID_0x212;
    break;
  case 0x213:
    msg.can_id = ID_0x213;
    break;
  case 0x214:
    msg.can_id = ID_0x214;
    break;
  case 0x215:
    msg.can_id = ID_0x215;
    break;
  case 0x216:
    msg.can_id = ID_0x216;
    break;
  }

  msg.dlc = CAN.packetDlc();
  for (int i = 0; i < msg.dlc; i++)
  {
    while (CAN.available())
    {
      msg.data[i] = CAN.read();
      break;
    }
  }
  rx_msg_buffer[buffer_wp] = msg;
  buffer_wp++;
  buffer_wp %= CAN_RX_BUFFER_SIZE;
}

void setup()
{
  // put your setup code here, to run once:

  M5.begin();
  PS4.begin("8C:AA:B5:81:71:BA");
  // ESP32 Timer
  initializeTimer();

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3))
  {
    // header("Starting CAN failed!", BLACK);
    while (1)
      ;
  }
  volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;

  CAN.onReceive(onReceive);

  // multi task
  //xTaskCreatePinnedToCore(Core0a, "Core0a", 8192, NULL, 2, &thp[0], 0);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (robomasters1_can_timeout) // Timeout
  {
    delay(100);
    return;
  }

  // Initial Command 10sec after boot
  if (initial_task_number < 3)
  {
    // Initial Task 1
    if (timer100msec_flag == 1 && initial_task_number == 0)
    {
      initial_task_number = 1;
      timer100msec_flag = 0;

      // boot command
      for (int command_no = 26; command_no < 35; command_no++)
      {
        uint8_t header_command[0xFF];
        uint8_t idx = 0;
        uint8_t command_length = can_command_list[command_no][3];
        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }

          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);
        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }
      }
    }

    // Initial Task 2
    if (timer100msec_flag == 1 && initial_task_number == 1)
    {
      initial_task_number = 2;
      timer100msec_flag = 0;

      // LED Flash
      {
        uint8_t command_no = 12;
        uint8_t header_command[0xFF];
        uint8_t idx = 0;
        uint8_t command_length = can_command_list[command_no][3];
        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = counter_led & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (counter_led >> 8) & 0xFF;
          }
          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);

        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }

        counter_led++;
      }

      // FAST MODE
      {
        uint8_t command_no = 22;
        uint8_t header_command[0xFF];
        uint8_t idx = 0;
        uint8_t command_length = can_command_list[command_no][3];
        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = counter_mode & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (counter_mode >> 8) & 0xFF;
          }
          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);

        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }

        counter_mode++;
      }
    }

    // Initial Task 3
    if (timer100msec_flag == 1 && initial_task_number == 2)
    {
      initial_task_number = 3;
      timer100msec_flag = 0;

      // Enable Odometry
      int command_no = 35;
      uint8_t header_command[0xFF];
      uint8_t idx = 0;
      uint8_t command_length = can_command_list[command_no][3];
      for (int i = 2; i < command_length; i++)
      {
        header_command[idx] = can_command_list[command_no][i];

        if (i == 5 && can_command_list[command_no][5] == 0xFF)
        {
          appendCRC8CheckSum(header_command, 4);
        }

        idx++;
      }
      appendCRC16CheckSum(header_command, command_length);
      for (int i = 0; i < command_length; i++)
      {
        can_command_buffer[can_command_buffer_wp] = header_command[i];
        can_command_buffer_wp++;
        can_command_buffer_wp %= TX_BUFFER_SIZE;
      }
    }
  }

  // After Initialize Task
  if (initial_task_number == 3)
  {
    // 10msec TASK
    if (timer10msec_flag)
    {
      timer10msec_flag = 0;

      if (command_lose == 1) // Lose Command
      {
        command_lose = 0;
        // Lose command
        int command_no = 36;

        uint8_t header_command[0xFF];
        uint8_t idx = 0;
        uint8_t command_length = can_command_list[command_no][3];
        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = counter_lose & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (counter_lose >> 8) & 0xFF;
          }

          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);

        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }

        counter_lose++;
      }

      if (command_lose == 2) // Recoover Command
      {
        command_lose = 0;
        // Recover command
        int command_no = 37;

        uint8_t header_command[0xFF];
        uint8_t idx = 0;
        uint8_t command_length = can_command_list[command_no][3];
        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = counter_lose & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (counter_lose >> 8) & 0xFF;
          }

          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);

        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }

        counter_lose++;
      }

      // LED Command
      if (command_led.enable)
      {
        command_led.enable = 0;
        // LED command
        for (int command_no = 9; command_no < 11; command_no++)
        {
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (int i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_led & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_led >> 8) & 0xFF;
            }

            // RED
            if (i == 16)
            {
              header_command[idx] = command_led.red;
            }

            // GREEN
            if (i == 17)
            {
              header_command[idx] = command_led.green;
            }

            // BLUE
            if (i == 18)
            {
              header_command[idx] = command_led.blue;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= TX_BUFFER_SIZE;
          }

          counter_led++;
        }
      }

      // Blaster Command
      if (command_blaster)
      {
        command_blaster = 0;

        // Blaster
        for (int command_no = 7; command_no < 9; command_no++)
        {
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (int i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_blaster & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_blaster >> 8) & 0xFF;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= TX_BUFFER_SIZE;
          }

          counter_blaster++;
        }
      }

      // Gel Blaster Command
      if (command_gel_blaster)
      {
        command_gel_blaster = 0;

        // Gel Blaster
        for (int command_no = 38; command_no < 39; command_no++)
        {
          uint8_t header_command[0xFF];
          uint8_t idx = 0;
          uint8_t command_length = can_command_list[command_no][3];
          for (int i = 2; i < command_length; i++)
          {
            header_command[idx] = can_command_list[command_no][i];

            if (i == 5 && can_command_list[command_no][5] == 0xFF)
            {
              appendCRC8CheckSum(header_command, 4);
            }
            if (i == 8 && can_command_list[command_no][8] == 0xFF)
            {
              header_command[idx] = counter_gel_blaster & 0xFF;
            }
            if (i == 9 && can_command_list[command_no][9] == 0xFF)
            {
              header_command[idx] = (counter_gel_blaster >> 8) & 0xFF;
            }

            idx++;
          }
          appendCRC16CheckSum(header_command, command_length);

          for (int i = 0; i < command_length; i++)
          {
            can_command_buffer[can_command_buffer_wp] = header_command[i];
            can_command_buffer_wp++;
            can_command_buffer_wp %= TX_BUFFER_SIZE;
          }

          counter_gel_blaster++;
        }
      }

      // Twist Command
      {
        uint8_t command_no = 5;
        uint8_t command_length = can_command_list[command_no][3];
        uint8_t header_command[0xFF];
        uint8_t idx = 0;

        // Linear X and Y
        uint16_t linear_x = 1024;
        uint16_t linear_y = 1024;
        int16_t angular_z = 1024;
        if (command_vel.enable)
        {
          linear_x = 256 * command_vel.twist_linear.x + 1024;
          linear_y = 256 * command_vel.twist_linear.y + 1024;
          angular_z = 0;
          if (command_control_mode)
          {
            angular_z = 256 * command_vel.twist_angular.z + 1024;
          }
        }

        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = command_counter[command_no] & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
          }
          command_counter[i]++;

          switch (i)
          {
          case 15:
            header_command[idx] = can_command_list[command_no][i] & 0xC0;
            header_command[idx] |= (linear_x >> 5) & 0x3F;
            break;
          case 14:
            header_command[idx] = linear_x << 3;
            header_command[idx] |= (linear_y >> 8) & 0x07;
            break;
          case 13:
            header_command[idx] = linear_y & 0xFF;
            break;
          case 19:
            header_command[idx] = (angular_z >> 4) & 0xFF; // 0x40;
            break;
          case 18:
            header_command[idx] = (angular_z << 4) | 0x08; // 0x08;
            break;
          case 20:
            header_command[idx] = 0x00;
            break;
          case 21:
            header_command[idx] = 0x02 | ((angular_z << 2) & 0xFF);
            break;
          case 22:
            header_command[idx] = (angular_z >> 6) & 0xFF; // 0x10;
            break;
          case 23:
            header_command[idx] = 0x04;
            break;
          case 24:
            header_command[idx] = 0x0C; // Enable Flag 4:x-y 8:yaw 0x0c
            break;
          case 25:
            header_command[idx] = 0x00;
            break;
          case 26:
            header_command[idx] = 0x04;
            break;
          default:
            break;
          }

          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);
        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }
      }

      // Gimbal Command
      {
        uint8_t command_no = 4;
        uint8_t command_length = can_command_list[command_no][3];
        uint8_t header_command[0xFF];
        uint8_t idx = 0;

        // Angular X and Y
        int16_t angular_y = 0;
        int16_t angular_z = 0;
        if (command_vel.enable)
        {
          angular_y = -1024 * command_vel.twist_angular.y;
          angular_z = -1024 * command_vel.twist_angular.z;
        }

        for (int i = 2; i < command_length; i++)
        {
          header_command[idx] = can_command_list[command_no][i];

          if (i == 5 && can_command_list[command_no][5] == 0xFF)
          {
            appendCRC8CheckSum(header_command, 4);
          }
          if (i == 8 && can_command_list[command_no][8] == 0xFF)
          {
            header_command[idx] = command_counter[command_no] & 0xFF;
          }
          if (i == 9 && can_command_list[command_no][9] == 0xFF)
          {
            header_command[idx] = (command_counter[command_no] >> 8) & 0xFF;
          }
          command_counter[command_no]++;

          switch (i)
          {
          case 16:
            header_command[idx] = (angular_y >> 8) & 0xFF;
            break;
          case 15:
            header_command[idx] = angular_y & 0xFF;
            break;
          case 18:
            header_command[idx] = (angular_z >> 8) & 0xFF;
            break;
          case 17:
            header_command[idx] = angular_z & 0xFF;
            break;
          default:
            break;
          }

          idx++;
        }
        appendCRC16CheckSum(header_command, command_length);
        for (int i = 0; i < command_length; i++)
        {
          can_command_buffer[can_command_buffer_wp] = header_command[i];
          can_command_buffer_wp++;
          can_command_buffer_wp %= TX_BUFFER_SIZE;
        }
      }

      // Register 10msec Task
      for (int i = 0; i < COMMAND_LIST_SIZE; i++)
      {
        if (can_command_list[i][1] == 1 && i != 5 && i != 4)
        {
          set_can_command((uint8_t)i);
        }
      }
    }

    // 100msec TASK
    if (timer100msec_flag)
    {
      timer100msec_flag = 0;

      for (int i = 0; i < COMMAND_LIST_SIZE; i++)
      {
        if (can_command_list[i][1] == 10)
        {
          set_can_command((uint8_t)i);
        }
      }
    }

    // 1sec TASK
    if (timer1sec_flag)
    {
      timer1sec_flag = 0;

      for (int i = 0; i < COMMAND_LIST_SIZE; i++)
      {
        if (can_command_list[i][1] == 100)
        {
          set_can_command((uint8_t)i);
        }
      }
    }

    // 10sec TASK
    if (timer10sec_flag)
    {
      timer10sec_flag = 0;
    }
  }

  // Transmit CAN Command
  while (can_command_buffer_rp != can_command_buffer_wp)
  {
    uint8_t TxData[8];
    uint8_t dlc = 0;

    int can_command_buffer_size = (can_command_buffer_wp - can_command_buffer_rp + TX_BUFFER_SIZE) % TX_BUFFER_SIZE;
    if (can_command_buffer_size >= 8)
    {
      dlc = 8;
    }
    else
    {
      dlc = can_command_buffer_size;
    }
    CAN.beginPacket(0x201);
    for (int i = 0; i < dlc; i++)
    {
      CAN.write(can_command_buffer[(can_command_buffer_rp + i) % TX_BUFFER_SIZE]);
    }
    // Start Transmission process
    CAN.endPacket();
    can_command_buffer_rp += dlc;
    can_command_buffer_rp %= TX_BUFFER_SIZE;
  }

  // Receive from RoboMaster S1
  while (buffer_rp != buffer_wp)
  {
    CANRxMsg msg;
    msg = rx_msg_buffer[buffer_rp];
    int ret = parseCanData(msg.can_id, msg.data, msg.dlc, received_data, &received_data_size);
    buffer_rp++;
    buffer_rp %= CAN_RX_BUFFER_SIZE;
    if (ret)
    {
      robomasters1_can_timeout_count = ROBOMASTER_S1_CAN_TIMEOUT;
      switch (msg.can_id)
      {

      // Motion Controller Output Data
      case ID_0x202:
      {
        if (received_data[1] == 0x3D && received_data[4] == 0x03 && received_data[5] == 0x09)
        {
          int flag = (received_data[24] >> 7) & 0x01;
          base_odom_yaw = ((((uint16_t)received_data[24]) << 8) | (((uint16_t)received_data[23]) << 0));
          if (flag == 0) // minus
          {
            int shift = (0x86 - ((received_data[24] << 1) | (received_data[23] >> 7)));
            base_odom_yaw = ((1 << 7) | received_data[23]) >> shift;
            base_odom_yaw *= -1;
          }
          else // plus
          {
            int shift = (0x186 - ((received_data[24] << 1) | (received_data[23] >> 7)));
            base_odom_yaw = ((1 << 7) | received_data[23]) >> shift;
          }
          if (received_data[24] == 0 && received_data[23] == 0)
          {
            base_odom_yaw = 0;
          }

          // unknown angle filtered value
          float_uint8 base_odom_yaw_union[5];
          base_odom_yaw_union[0].uint8_data[0] = received_data[35];
          base_odom_yaw_union[0].uint8_data[1] = received_data[36];
          base_odom_yaw_union[0].uint8_data[2] = received_data[37];
          base_odom_yaw_union[0].uint8_data[3] = received_data[38];
          base_odom_yaw_union[1].uint8_data[0] = received_data[39];
          base_odom_yaw_union[1].uint8_data[1] = received_data[40];
          base_odom_yaw_union[1].uint8_data[2] = received_data[41];
          base_odom_yaw_union[1].uint8_data[3] = received_data[42];
          base_odom_yaw_union[2].uint8_data[0] = received_data[43];
          base_odom_yaw_union[2].uint8_data[1] = received_data[44];
          base_odom_yaw_union[2].uint8_data[2] = received_data[45];
          base_odom_yaw_union[2].uint8_data[3] = received_data[46];
          base_odom_yaw_union[3].uint8_data[0] = received_data[47];
          base_odom_yaw_union[3].uint8_data[1] = received_data[48];
          base_odom_yaw_union[3].uint8_data[2] = received_data[49];
          base_odom_yaw_union[3].uint8_data[3] = received_data[50];
          base_odom_yaw_union[4].uint8_data[0] = received_data[51];
          base_odom_yaw_union[4].uint8_data[1] = received_data[52];
          base_odom_yaw_union[4].uint8_data[2] = received_data[53];
          base_odom_yaw_union[4].uint8_data[3] = received_data[54];
          filtered_vel_x1 = base_odom_yaw_union[0].float_data;
          filtered_vel_y1 = base_odom_yaw_union[1].float_data;
          unknown_data = base_odom_yaw_union[2].float_data;
          filtered_vel_x2 = base_odom_yaw_union[3].float_data;
          filtered_vel_y1 = base_odom_yaw_union[4].float_data;
        }

        if (received_data[1] == 0x31 && received_data[4] == 0x03 && received_data[5] == 0x04)
        {
          // ~20 is unknown counter and state
          float_uint8 quaternion_union[4];
          quaternion_union[0].uint8_data[0] = received_data[21];
          quaternion_union[0].uint8_data[1] = received_data[22];
          quaternion_union[0].uint8_data[2] = received_data[23];
          quaternion_union[0].uint8_data[3] = received_data[24];
          quaternion_union[1].uint8_data[0] = received_data[25];
          quaternion_union[1].uint8_data[1] = received_data[26];
          quaternion_union[1].uint8_data[2] = received_data[27];
          quaternion_union[1].uint8_data[3] = received_data[28];
          quaternion_union[2].uint8_data[0] = received_data[29];
          quaternion_union[2].uint8_data[1] = received_data[30];
          quaternion_union[2].uint8_data[2] = received_data[31];
          quaternion_union[2].uint8_data[3] = received_data[32];
          quaternion_union[3].uint8_data[0] = received_data[33];
          quaternion_union[3].uint8_data[1] = received_data[34];
          quaternion_union[3].uint8_data[2] = received_data[35];
          quaternion_union[3].uint8_data[3] = received_data[36];
          base_pose_quaternion.w = quaternion_union[0].float_data;
          base_pose_quaternion.x = quaternion_union[1].float_data;
          base_pose_quaternion.y = quaternion_union[2].float_data;
          base_pose_quaternion.z = quaternion_union[3].float_data;

          uint16_t uint16_data = received_data[38];
          uint16_data = (uint16_data << 8) | received_data[37];
          voltage = (int16_t)uint16_data;

          uint16_data = received_data[40];
          uint16_data = (uint16_data << 8) | received_data[39];
          temperature = (int16_t)uint16_data;

          uint32_t uint32_data = received_data[44];
          uint32_data = (uint32_data << 8) | received_data[43];
          uint32_data = (uint32_data << 8) | received_data[42];
          uint32_data = (uint32_data << 8) | received_data[41];
          current = -(int32_t)uint32_data;

          battery_soc = received_data[45];
        }

        // Counter
        if (received_data[1] == 0x39 && received_data[4] == 0x03 && received_data[5] == 0x0A)
        {
          uint32_t counter = 0;
          counter = (received_data[14] & 0x0F) * 10000;
          counter += (received_data[15] & 0x0F) * 1000;
          counter += (received_data[16] & 0x0F) * 100;
          counter += (received_data[17] & 0x0F) * 10;
          counter += (received_data[18] & 0x0F) * 1;
        }

        // Odometry Lab Mode Data
        if (received_data[1] == 0x6F && received_data[4] == 0x03 && received_data[5] == 0x09)
        {
          // Odom in Lab mode
          uint32_t data;
          int32_t int_data;
          uint16_t data16;
          int16_t int_data16;

          data = received_data[16];
          data = (data << 8) | received_data[15];
          data = (data << 8) | received_data[14];
          data = (data << 8) | received_data[13];
          odom_counter = (uint32_t)data;

          data = received_data[20];
          data = (data << 8) | received_data[19];
          data = (data << 8) | received_data[18];
          data = (data << 8) | received_data[17];
          odom_counter2 = (uint32_t)data;

          // Wheel Angular Velocity
          // Unit is RPM
          data16 = received_data[22];
          data16 = (data16 << 8) | received_data[21];
          wheel_angular_velocity[0] = (int16_t)data16;

          data16 = received_data[24];
          data16 = (data16 << 8) | received_data[23];
          wheel_angular_velocity[1] = (int16_t)data16;

          data16 = received_data[26];
          data16 = (data16 << 8) | received_data[25];
          wheel_angular_velocity[2] = (int16_t)data16;

          data16 = received_data[28];
          data16 = (data16 << 8) | received_data[27];
          wheel_angular_velocity[3] = (int16_t)data16;

          // wheel position
          data16 = received_data[30];
          data16 = (data16 << 8) | received_data[29];
          data16 = data16 << 1;
          int_data16 = (int16_t)data16;
          wheel_angle[0] = int_data16 / 32767.0 * 180.0;

          data16 = received_data[32];
          data16 = (data16 << 8) | received_data[31];
          data16 = data16 << 1;
          int_data16 = (int16_t)data16;
          wheel_angle[1] = int_data16 / 32767.0 * 180.0;

          data16 = received_data[34];
          data16 = (data16 << 8) | received_data[33];
          data16 = data16 << 1;
          int_data16 = (int16_t)data16;
          wheel_angle[2] = int_data16 / 32767.0 * 180.0;

          data16 = received_data[36];
          data16 = (data16 << 8) | received_data[35];
          data16 = data16 << 1;
          int_data16 = (int16_t)data16;
          wheel_angle[3] = int_data16 / 32767.0 * 180.0;

          m_bus_update_count[4];
          // wheel update count
          data = received_data[40];
          data = (data << 8) | received_data[39];
          data = (data << 8) | received_data[38];
          data = (data << 8) | received_data[37];
          m_bus_update_count[0] = (uint32_t)data;

          data = received_data[44];
          data = (data << 8) | received_data[43];
          data = (data << 8) | received_data[42];
          data = (data << 8) | received_data[41];
          m_bus_update_count[1] = (uint32_t)data;

          data = received_data[48];
          data = (data << 8) | received_data[47];
          data = (data << 8) | received_data[46];
          data = (data << 8) | received_data[45];
          m_bus_update_count[2] = (uint32_t)data;

          data = received_data[52];
          data = (data << 8) | received_data[51];
          data = (data << 8) | received_data[50];
          data = (data << 8) | received_data[49];
          m_bus_update_count[3] = (uint32_t)data;

          // IMU Data
          float_uint8 union_data;

          union_data.uint8_data[0] = received_data[53 + 4];
          union_data.uint8_data[1] = received_data[54 + 4];
          union_data.uint8_data[2] = received_data[55 + 4];
          union_data.uint8_data[3] = received_data[56 + 4];
          mag_x = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 2];
          union_data.uint8_data[1] = received_data[54 + 4 * 2];
          union_data.uint8_data[2] = received_data[55 + 4 * 2];
          union_data.uint8_data[3] = received_data[56 + 4 * 2];
          mag_y = union_data.float_data;

          // mag_z is blank

          union_data.uint8_data[0] = received_data[53 + 4 * 4];
          union_data.uint8_data[1] = received_data[54 + 4 * 4];
          union_data.uint8_data[2] = received_data[55 + 4 * 4];
          union_data.uint8_data[3] = received_data[56 + 4 * 4];
          g_x = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 5];
          union_data.uint8_data[1] = received_data[54 + 4 * 5];
          union_data.uint8_data[2] = received_data[55 + 4 * 5];
          union_data.uint8_data[3] = received_data[56 + 4 * 5];
          g_y = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 6];
          union_data.uint8_data[1] = received_data[54 + 4 * 6];
          union_data.uint8_data[2] = received_data[55 + 4 * 6];
          union_data.uint8_data[3] = received_data[56 + 4 * 6];
          g_z = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 7];
          union_data.uint8_data[1] = received_data[54 + 4 * 7];
          union_data.uint8_data[2] = received_data[55 + 4 * 7];
          union_data.uint8_data[3] = received_data[56 + 4 * 7];
          gyro_x = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 8];
          union_data.uint8_data[1] = received_data[54 + 4 * 8];
          union_data.uint8_data[2] = received_data[55 + 4 * 8];
          union_data.uint8_data[3] = received_data[56 + 4 * 8];
          gyro_y = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 9];
          union_data.uint8_data[1] = received_data[54 + 4 * 9];
          union_data.uint8_data[2] = received_data[55 + 4 * 9];
          union_data.uint8_data[3] = received_data[56 + 4 * 9];
          gyro_z = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 11];
          union_data.uint8_data[1] = received_data[54 + 4 * 11];
          union_data.uint8_data[2] = received_data[55 + 4 * 11];
          union_data.uint8_data[3] = received_data[56 + 4 * 11];
          yaw = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 12];
          union_data.uint8_data[1] = received_data[54 + 4 * 12];
          union_data.uint8_data[2] = received_data[55 + 4 * 12];
          union_data.uint8_data[3] = received_data[56 + 4 * 12];
          pitch = union_data.float_data;

          union_data.uint8_data[0] = received_data[53 + 4 * 13];
          union_data.uint8_data[1] = received_data[54 + 4 * 13];
          union_data.uint8_data[2] = received_data[55 + 4 * 13];
          union_data.uint8_data[3] = received_data[56 + 4 * 13];
          roll = union_data.float_data;
        }

        break;
      }

      // Gimbal Output Data
      case ID_0x203: // 0x203
      {
        // From Gimbal
        if (received_data[1] == 0x11 && received_data[4] == 0x04 && received_data[5] == 0x03)
        {
          // Pan Angle
          uint16_t data = received_data[12];
          data = (data << 8) | received_data[11];
          gimbal_base_pan_angle = -(int16_t)(data) / 10.0 * 3.141592 / 180;
          data = received_data[14];
          data = (data << 8) | received_data[13];
          gimbal_map_pan_angle = -(int16_t)(data) / 100.0 * 3.141592 / 180;
        }
        if (received_data[1] == 0x16 && received_data[4] == 0x04 && received_data[5] == 0x09)
        {
          // Tilt Angle
          uint32_t data = received_data[14];
          data = (data << 8) | received_data[13];
          data = (data << 8) | received_data[12];
          data = (data << 8) | received_data[11];
          gimbal_map_tilt_angle = (-(int32_t)(data) / 65536.0 / 10 - 7.6) * 3.141592 / 180;
          data = received_data[18];
          data = (data << 8) | received_data[17];
          data = (data << 8) | received_data[16];
          data = (data << 8) | received_data[15];
          gimbal_base_tilt_angle = (-(int32_t)(data) / 65536.0 / 10 - 7.6) * 3.141592 / 180;
          // printf("%lf, %lf\n", gimbal_map_tilt_angle_, gimbal_base_tilt_angle_);
        }
        break;
      }
      case ID_0x204: // 0x204
      {
        if (received_data[4] == 0x17 && received_data[5] == 0x09)
        {
        }
        break;
      }
      case ID_0x211: // BACK SENSOR
      {
        if (received_data[4] == 0x38 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      case ID_0x212: // FRONT SENSOR
      {
        if (received_data[4] == 0x58 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      case ID_0x213: // LEFT SENSOR
      {
        if (received_data[4] == 0x78 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      case ID_0x214: // RIGHT SENSOR
      {
        if (received_data[4] == 0x98 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      case ID_0x215: // GIMBAL SENSOR LEFT
      {
        if (received_data[4] == 0xB8 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      case ID_0x216: // GIMBAL SENSOR RIGHT
      {
        if (received_data[4] == 0xD8 && received_data[5] == 0x09) // or 1D
        {
          PS4.setRumble(100, 200);
          // M5.Lcd.printf("0x%02X\n",received_data[5]);

          // Sends data set in the above three instructions to the controller
          PS4.sendToController();
        }
        break;
      }
      default:
        break;
      }

      break;
    }
  }

  // Get PS4 Key
  if (PS4.isConnected())
  {
    if (PS4.Cross())
    {
      command_blaster = 1;
    }
    if (PS4.Circle())
    {
      command_gel_blaster = 1;
    }
    if (PS4.Triangle())
    {
      command_lose = 1;
    }
    if (PS4.Square())
    {
      command_lose = 2;
    }

    if (PS4.R2())
    {
      command_led = led_list[led_list_count];
      led_list_count++;
      led_list_count %= LED_LIST_NUM;
    }
    if (PS4.L2())
    {
      command_led = led_list[led_list_count];
      led_list_count--;
      led_list_count %= LED_LIST_NUM;
    }

    if (PS4.R1())
    {
      command_control_mode = 0;
    }
    else
    {
      command_control_mode = 1;
    }

    if (PS4.L1())
    {
      // M5.Lcd.printf("X: %d",PS4.LStickX()); // -128 to 128
      command_vel_timeout_count = COMMAND_VEL_TIMEOUT;
      command_vel.enable = true;
      command_vel.twist_linear.x = PS4.LStickY() / 256.0;
      command_vel.twist_linear.y = PS4.LStickX() / 256.0;
      command_vel.twist_angular.y = PS4.RStickY() / 256.0 / 10.0;
      command_vel.twist_angular.z = -PS4.RStickX() / 256.0 / 10.0;
    }
    else
    {
      command_vel_timeout_count = COMMAND_VEL_TIMEOUT;
      command_vel.enable = false;
      command_vel.twist_linear.x = 0;
      command_vel.twist_linear.y = 0;
      command_vel.twist_angular.y = 0;
      command_vel.twist_angular.z = 0;
    }
    // if (PS4.LStickY()) {
    //   M5.Lcd.printf("Hello World");
    // }
  }
  delayMicroseconds(10);
}
