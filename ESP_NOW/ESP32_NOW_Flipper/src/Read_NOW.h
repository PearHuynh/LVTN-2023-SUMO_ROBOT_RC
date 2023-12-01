#ifndef READ_NOW_H
#define READ_NOW_H

#include <Arduino.h>
#include "CoESumoRobot.h"
#include <Servo.h>

uint8_t broadcastAddress[] = {0xC4, 0xDE, 0xE2, 0x0E, 0xCE, 0x08}; // Add Sender =   C4:DE:E2:0E:CE:08

#define relay_pin IO_18
#define key_pin IO_32

/*--------------------------------------------------------------------//
  *  Structure example to receive data
  *  Must match the sender structure
//--------------------------------------------------------------------*/
typedef struct struc_read_data
{
  int left_JoyX_value = 0;
  int left_JoyY_value = 0;
  uint8_t left_JoySW_value = 0;
  uint8_t left_Button_A_value = 0;
  uint8_t left_Button_B_value = 0;

  int right_JoyX_value = 0;
  int right_JoyY_value = 0;
  uint8_t right_JoySW_value = 0;
  uint8_t right_Button_A_value = 0;
  uint8_t right_Button_B_value = 0;
} struc_read_data;

// Create a struct_message called read_data
static struc_read_data read_data;

/*--------------------------------------------------------------------//
  *  State LED, connect, Set motor, servo.
//--------------------------------------------------------------------*/
uint8_t State_connect = 0;
uint8_t LED_State = 0;
int8_t DIR = -1;
int8_t DIR_LR = -1;
int8_t DIR_xoay = -1;

int16_t SPEED = 0;
int16_t SPEED_LR = 0;
uint16_t xoay_tron = 0;

// uint8_t sp_Servo1 = 0;

uint8_t BT_Left = 0;
uint8_t BT_Right = 0;
uint8_t BT1 = 0;
uint8_t BT2 = 0;
uint8_t BT3 = 0;
uint8_t BT4 = 0;

uint8_t START = 0;
uint8_t pass1 = 0, pass2 = 0;
uint8_t buzzer_warning = 1;

// Servo servo1;
// Servo servo2;
// const uint8_t channal_Servo1 = 0;
// const uint8_t channal_Servo2 = 1;

void buzzer(uint8_t pinBuzzer, int delay_s)
{
  digitalWrite(pinBuzzer, LOW);
  delay(delay_s);
  digitalWrite(pinBuzzer, HIGH);
}

void on_kit()
{
  pinMode(IO_17, OUTPUT);
  digitalWrite(17, HIGH);
  delay(2000);
  digitalWrite(17, LOW);

  // warning on kit
  buzzer(IO_16, 100);
  delay(100);
  buzzer(IO_16, 100);
}

// void setup_servo()
// {
//   // Attach Servo
//   servo1.attach(IO_18, channal_Servo1);
//   servo2.attach(IO_19, channal_Servo2);

//   servo1.write(0);
//   servo2.write(0);
// }

void control_motor()
{
  // Thuc hien lenh chay dong co.
  if (DIR == 1)
  { // Chay tien
    CoEMaker.Set_Motor_Speed(MOTOR_L, SPEED);
    CoEMaker.Set_Motor_Speed(MOTOR_R, SPEED);
    if ((DIR_LR == 1) && (SPEED_LR > 0))
    { // Chay tien re phai
      CoEMaker.Set_Motor_Speed(MOTOR_L, SPEED_LR);
      CoEMaker.Set_Motor_Speed(MOTOR_R, SPEED_LR / 3);
    }
    else if ((DIR_LR == 0) && SPEED_LR > 0)
    { // Chay tien re trai
      CoEMaker.Set_Motor_Speed(MOTOR_L, SPEED_LR / 3);
      CoEMaker.Set_Motor_Speed(MOTOR_R, SPEED_LR);
    }
  }
  else if (DIR == 0)
  { // Chay lui
    CoEMaker.Set_Motor_Speed(MOTOR_L, -SPEED);
    CoEMaker.Set_Motor_Speed(MOTOR_R, -SPEED);
    if ((DIR_LR == 1) && (SPEED_LR) > 0)
    { // Chay lui re phai
      CoEMaker.Set_Motor_Speed(MOTOR_L, -SPEED_LR);
      CoEMaker.Set_Motor_Speed(MOTOR_R, -SPEED_LR / 3);
    }
    else if ((DIR_LR == 0) && SPEED_LR > 0)
    { // Chay lui re trai
      CoEMaker.Set_Motor_Speed(MOTOR_L, -SPEED_LR / 3);
      CoEMaker.Set_Motor_Speed(MOTOR_R, -SPEED_LR);
    }
  }
  else
  {
    if (DIR_LR == 1 && SPEED_LR > 0)
    { // Dung quay trai tai cho
      CoEMaker.Set_Motor_Speed(MOTOR_L, SPEED_LR);
      CoEMaker.Set_Motor_Speed(MOTOR_R, 0);
    }
    else if (DIR_LR == 0 && SPEED_LR > 0)
    { // Dung quay phai tai cho
      CoEMaker.Set_Motor_Speed(MOTOR_L, 0);
      CoEMaker.Set_Motor_Speed(MOTOR_R, SPEED_LR);
    }
    else if (DIR_xoay == 1 && xoay_tron > 0)
    { // Dung quay phai tai cho
      CoEMaker.Set_Motor_Speed(MOTOR_L, xoay_tron);
      CoEMaker.Set_Motor_Speed(MOTOR_R, -xoay_tron);
    }
    else if (DIR_xoay == 0 && xoay_tron > 0)
    { // Dung quay phai tai cho
      CoEMaker.Set_Motor_Speed(MOTOR_L, -xoay_tron);
      CoEMaker.Set_Motor_Speed(MOTOR_R, xoay_tron);
    }
    else
    {
      CoEMaker.Motor_Stop_Short_brake();
    }
  }
}

void Xem_data_read_espnow()
{
  // Xem trang thai 6 nut nhan
  if (read_data.left_JoySW_value)
  {
    Serial.println("left_JoySW_value.");
  }
  if (read_data.left_Button_A_value)
  {
    Serial.println("left_Button_A_value.");
  }
  if (read_data.left_Button_B_value)
  {
    Serial.println("left_Button_B_value.");
  }
  if (read_data.right_JoySW_value)
  {
    Serial.println("right_JoySW_value.");
  }
  if (read_data.right_Button_A_value)
  {
    Serial.println("right_Button_A_value.");
  }
  if (read_data.right_Button_B_value)
  {
    Serial.println("right_Button_B_value.");
  }

  // Xem trang thai doc JoysTick Left
  // Serial.printf("LEFT: RVX: %d, RVY: %d, SM: %d, BTA: %d, BTB: %d ",
  //               read_data.left_JoyX_value,
  //               read_data.left_JoyY_value,
  //               read_data.left_JoySW_value,
  //               read_data.left_Button_A_value,
  //               read_data.left_Button_B_value);

  // // Xem trang thai doc JoysTick Right
  // Serial.printf("----- RIGH: RVX: %d, RVY: %d, SM: %d, BTA: %d, BTB: %d \n",
  //               read_data.right_JoyX_value,
  //               read_data.right_JoyY_value,
  //               read_data.right_JoySW_value,
  //               read_data.right_Button_A_value,
  //               read_data.right_Button_B_value);

  Serial.printf("DRI: %d, SPEED: %d, DIR_LR: %d, SPEED_LR: %d \n", DIR, SPEED, DIR_LR, SPEED_LR);
}

#endif
