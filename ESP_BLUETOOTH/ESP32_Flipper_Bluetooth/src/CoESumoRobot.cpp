#include "CoESumoRobot.h"

void CoESumoRobot::begin() {
  //Khai bao PWM
  ledcSetup(channal_PWM_L, freq_15kHz, resolution);
  ledcSetup(channal_PWM_R, freq_15kHz, resolution);

  //Attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_L_PIN, channal_PWM_L);
  ledcAttachPin(PWM_R_PIN, channal_PWM_R);

  //OUTPUT Driver
  pinMode(DIR1_L_PIN, OUTPUT);
  pinMode(DIR2_L_PIN, OUTPUT);
  pinMode(DIR1_R_PIN, OUTPUT);
  pinMode(DIR2_R_PIN, OUTPUT);

  //Display
  pinMode(LED_ESP, OUTPUT);
  pinMode(LED_USER, OUTPUT);

  //Read INPUT
  pinMode(BT_START_PIN, INPUT);
  pinMode(DIP_SWITCH_PIN, INPUT);

  //Set trang thai ban dau.
  digitalWrite(DIR1_L_PIN, LOW);
  digitalWrite(DIR2_L_PIN, LOW);
  digitalWrite(DIR1_R_PIN, LOW);
  digitalWrite(DIR2_R_PIN, LOW);
}

/*Read DIP Switch with ADC => MODE 0 to 7*/
int CoESumoRobot::Read_DIP_Switch() {
  int dip_switch_adc;
  for (int i = 0; i < 10; i++) {
    dip_switch_adc = simpleKalmanFilter.updateEstimate(analogRead(DIP_SWITCH_PIN));
    // Serial.println("Chua loc" + String(dip_switch_adc));
  }
  // Serial.println("Da thuc hien loc: " + String(dip_switch_adc));
  if (dip_switch_adc > 3650) return 7;
  if (dip_switch_adc > 3550) return 6;
  if (dip_switch_adc > 3410) return 5;
  if (dip_switch_adc > 3250) return 4;
  if (dip_switch_adc > 2940) return 3;
  if (dip_switch_adc > 2610) return 2;
  if (dip_switch_adc > 1860) return 1;
  return 0;
}

/*Read button START in while and delay with user set time*/
bool CoESumoRobot::Read_START(uint8_t timedelay) {  //delay before START
  if (digitalRead(BT_START_PIN) == 0) {
    CoEMaker.PWM_Write(LED_USER, 0);
    while (digitalRead(BT_START_PIN) == 0) {}
    delay(timedelay * 1000);
    CoEMaker.PWM_Write(LED_USER, 1023);
    return 1;
  } else {
    CoEMaker.PWM_Write(LED_USER, 1023);
    delay(100);
    CoEMaker.PWM_Write(LED_USER, 0);
    delay(100);
  }
  return 0;
}

/*Using PWM with library ledc*/
void CoESumoRobot::PWM_Write(uint8_t pinPWM, int size) {
  switch (pinPWM) {
    case PWM_L_PIN:
      ledcWrite(channal_PWM_L, size);
      break;
    case PWM_R_PIN:
      ledcWrite(channal_PWM_R, size);
      break;
  }
}

/*Map speed 0 -> 10 is equal to 0 -> 1023*/
uint16_t CoESumoRobot::Map_Speed(uint8_t speed_direction, int8_t speed10) {  //motor left = 0; right = 1;
  if (speed10 >= 0 && speed10 <= 10) {
    if (speed_direction) {
      return SPEED_R[speed10];
    } else {
      return SPEED_L[speed10];
    }
  } else {
    return 0;
  }
}

/*Motor stop*/
void CoESumoRobot::Motor_Stop() {
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  digitalWrite(DIR1_L_PIN, LOW);
  digitalWrite(DIR2_L_PIN, LOW);

  CoEMaker.PWM_Write(PWM_R_PIN, 0);
  digitalWrite(DIR1_R_PIN, LOW);
  digitalWrite(DIR2_R_PIN, LOW);
}

/*Motor stop*/
void CoESumoRobot::Motor_Stop_Short_brake() {
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  digitalWrite(DIR1_L_PIN, HIGH);
  digitalWrite(DIR2_L_PIN, HIGH);

  CoEMaker.PWM_Write(PWM_R_PIN, 0);
  digitalWrite(DIR1_R_PIN, HIGH);
  digitalWrite(DIR2_R_PIN, HIGH);
}

/*Motor move forward with user set speed*/
void CoESumoRobot::Motor_Move_Forward(uint8_t speed) {  //Set speed 0 to 10
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  CoEMaker.PWM_Write(PWM_R_PIN, 0);

  digitalWrite(DIR1_L_PIN, HIGH);
  digitalWrite(DIR2_L_PIN, LOW);

  digitalWrite(DIR1_R_PIN, HIGH);
  digitalWrite(DIR2_R_PIN, LOW);

  CoEMaker.PWM_Write(PWM_L_PIN, CoEMaker.Map_Speed(MOTOR_L, speed));
  CoEMaker.PWM_Write(PWM_R_PIN, CoEMaker.Map_Speed(MOTOR_R, speed));
}

/*Motor move backward with user set speed*/
void CoESumoRobot::Motor_Move_Backward(uint8_t speed) {  //Set speed 0 to 10
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  CoEMaker.PWM_Write(PWM_R_PIN, 0);

  digitalWrite(DIR1_L_PIN, LOW);
  digitalWrite(DIR2_L_PIN, HIGH);

  digitalWrite(DIR1_R_PIN, LOW);
  digitalWrite(DIR2_R_PIN, HIGH);

  CoEMaker.PWM_Write(PWM_L_PIN, CoEMaker.Map_Speed(MOTOR_L, speed));
  CoEMaker.PWM_Write(PWM_R_PIN, CoEMaker.Map_Speed(MOTOR_R, speed));
}

/*Motor turn left with user set speed*/
void CoESumoRobot::Motor_Turn_Left(uint8_t speed) {
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  CoEMaker.PWM_Write(PWM_R_PIN, 0);

  digitalWrite(DIR1_L_PIN, HIGH);
  digitalWrite(DIR2_L_PIN, HIGH);

  digitalWrite(DIR1_R_PIN, HIGH);
  digitalWrite(DIR2_R_PIN, LOW);

  CoEMaker.PWM_Write(PWM_R_PIN, CoEMaker.Map_Speed(MOTOR_R, speed));
}

/*Motor turn right with user set speed*/
void CoESumoRobot::Motor_Turn_Right(uint8_t speed) {
  CoEMaker.PWM_Write(PWM_L_PIN, 0);
  CoEMaker.PWM_Write(PWM_R_PIN, 0);

  digitalWrite(DIR1_L_PIN, HIGH);
  digitalWrite(DIR2_L_PIN, LOW);

  digitalWrite(DIR1_R_PIN, HIGH);
  digitalWrite(DIR2_R_PIN, HIGH);

  CoEMaker.PWM_Write(PWM_L_PIN, CoEMaker.Map_Speed(MOTOR_L, speed));
}

/*set speed -10 to 10; -10:0 = backwarf; 0:10 = forward;*/
void CoESumoRobot::Set_Motor_Speed(uint8_t motor, int8_t speed) {  // 0 motor left; 1 motor right
  switch (motor) {
    case MOTOR_L:
      CoEMaker.PWM_Write(PWM_L_PIN, 0);
      if (speed > 0) {  //Forward
        digitalWrite(DIR1_L_PIN, HIGH);
        digitalWrite(DIR2_L_PIN, LOW);
        CoEMaker.PWM_Write(PWM_L_PIN, CoEMaker.Map_Speed(MOTOR_L, speed));
      } else if (speed < 0) {  //Backward
        digitalWrite(DIR1_L_PIN, LOW);
        digitalWrite(DIR2_L_PIN, HIGH);
        CoEMaker.PWM_Write(PWM_L_PIN, CoEMaker.Map_Speed(MOTOR_L, speed = -speed));
      } else {
        digitalWrite(DIR1_L_PIN, HIGH);
        digitalWrite(DIR2_L_PIN, HIGH);
      }
      break;
    case MOTOR_R:
      CoEMaker.PWM_Write(PWM_R_PIN, 0);
      if (speed > 0) {  //Forward
        digitalWrite(DIR1_R_PIN, HIGH);
        digitalWrite(DIR2_R_PIN, LOW);
        CoEMaker.PWM_Write(PWM_R_PIN, CoEMaker.Map_Speed(MOTOR_R, speed));
      } else if (speed < 0) {
        digitalWrite(DIR1_R_PIN, LOW);
        digitalWrite(DIR2_R_PIN, HIGH);
        CoEMaker.PWM_Write(PWM_R_PIN, CoEMaker.Map_Speed(MOTOR_R, speed = -speed));
      } else {
        digitalWrite(DIR1_R_PIN, HIGH);
        digitalWrite(DIR2_R_PIN, HIGH);
      }
      break;
  }
}

CoESumoRobot CoEMaker;
