#ifndef COE_SUMO_ROBOT_H
#define COE_SUMO_ROBOT_H

#include <Arduino.h>
#include <SimpleKalmanFilter.h>

#define MOTOR_L 0
#define MOTOR_R 1
#define LEFT 0
#define RIGHT 1
#define SERVO_LEFT 0
#define SERVO_RIGHT 1
#define ULTRA_LEFT 0
#define ULTRA_RIGHT 1

//Display
#define LED_ESP 2
#define LED_USER 12

//OUTPUT
#define PWM_L_PIN 25
#define DIR1_L_PIN 26
#define DIR2_L_PIN 27

#define PWM_R_PIN 22
#define DIR1_R_PIN 23
#define DIR2_R_PIN 21

//DIP Swith and Button
#define DIP_SWITCH_PIN 39
#define BT_START_PIN 5

//Edge Sensors
#define IO_32 32
#define IO_33 33

//Ultrasonic Sensors
#define IO_19 19
#define IO_18 18
#define IO_17 17
#define IO_16 16
#define IO_14 14
#define IO_13 13

//Opponent Sensors
#define IN_34 34
#define IN_35 35
#define IN_36 36

class CoESumoRobot {
  //PWM Output
  const uint16_t freq_15kHz = 15000;
  const uint8_t channal_PWM_L = 8;     //Channal 8 (0 to 16 channal)
  const uint8_t channal_PWM_R = 9;     //Channal 9 (0 to 16 channal)
  const uint8_t resolution = 10;

  const uint16_t SPEED_L[11] = { 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1023 };
  const uint16_t SPEED_R[11] = { 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1023 };

public:
  // Loc nhieu doc ADC SIP SW
  CoESumoRobot()
    : simpleKalmanFilter(2, 2, 0.01) {}
  void begin();
  int Read_DIP_Switch();
  bool Read_START(uint8_t timedelay);
  void PWM_Write(uint8_t pinPWM, int size);
  uint16_t Map_Speed(uint8_t speed_direction, int8_t speed10);
  void Motor_Stop();
  void Motor_Stop_Short_brake();
  void Motor_Move_Forward(uint8_t speed);
  void Motor_Move_Backward(uint8_t speed);
  void Motor_Turn_Left(uint8_t speed);
  void Motor_Turn_Right(uint8_t speed);
  void Set_Motor_Speed(uint8_t motor, int8_t speed);
private:
  SimpleKalmanFilter simpleKalmanFilter;
};

extern CoESumoRobot CoEMaker;

#endif