#include <Arduino.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include "Read_Bluetooth.h"

BluetoothSerial SerialBT;
uint8_t state_connect = 0;

uint8_t count0 = 0, count1 = 0;
bool isConnected = false;
bool buzzer_connect = true;

StaticJsonDocument<250> json_receiver;
String data_read = "";
String data_receiver = "";

uint8_t start_cam = 0;

void OnDataRecv();

/*------------------------------------TASK CORE 0--------------------------------------------*/
TaskHandle_t Task1;
void Task1code_0(void *parameter)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  pinMode(relay_pin, OUTPUT);
  pinMode(key_pin, INPUT);
  digitalWrite(relay_pin, HIGH);
  uint16_t count_out = 0;
  while (1)
  {
    if (BT3 && START)
    {
      Serial.println("Start cam");
      while (digitalRead(key_pin) == 0)
      {
        digitalWrite(relay_pin, LOW);
        if (++count_out > 1000)
        {
          count_out = 0;
          break;
        }
        delay(1);
      }
      BT3 = 0;
    }
    else if (BT4 && START)
    {
      Serial.println("Set cam");
      while (digitalRead(key_pin) == 1)
      {
        digitalWrite(relay_pin, LOW);
        if (++count_out > 1000)
        {
          count_out = 0;
          break;
        }
        delay(1);
      }
      BT4 = 0;
    }
    else
    {
      digitalWrite(relay_pin, HIGH);
    }

    delay(10);
  }
}

void read_BT(){
  if (SerialBT.available()){
    while (SerialBT.available()){
      char inChar = (char)SerialBT.read();
      if (inChar != '\n'){
      data_receiver += inChar;
      } else {
        // Serial.println(data_receiver);
        deserializeJson(json_receiver, data_receiver);
        read_data.left_JoyX_value = json_receiver["LX"];
        read_data.left_JoyY_value = json_receiver["LY"];
        read_data.left_JoySW_value = json_receiver["LW"];
        read_data.left_Button_A_value = json_receiver["LA"];
        read_data.left_Button_B_value = json_receiver["LB"];
        read_data.right_JoyX_value = json_receiver["RX"];
        read_data.right_JoyY_value = json_receiver["RY"];
        read_data.right_JoySW_value = json_receiver["RW"];
        read_data.right_Button_A_value = json_receiver["RA"];
        read_data.right_Button_B_value = json_receiver["RB"];
        read_data.status = json_receiver["ST"];
        OnDataRecv();
        data_receiver = "";
        SerialBT.println("OK");
        }
      }
    }  
         
    if(read_data.status == 0){
      count0++;
      count1 = 0;
      if(count0 > 10) count0 = 10;
    } else {
      count1++;
      count0 = 0;
      if(count1 > 10) count1 = 10;
    }
    
    if(count1 > 9 || count0 > 9){
      isConnected = false;
    } else {
      isConnected = true;
    }
}

void setup()
{
  // Set buzzer
  pinMode(IO_16, OUTPUT);
  digitalWrite(IO_16, HIGH);

  // Set on kit
  on_kit();

  // Initialize Serial Monitor
  Serial.begin(115200);

  Serial.print("Main running on core ");
  Serial.println(xPortGetCoreID());

  // Set kit sumo robot
  CoEMaker.begin();

  SerialBT.begin("SUMO_ESP32_FLIPPER");

  // Setup start core 0
  xTaskCreatePinnedToCore(
      Task1code_0, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      1,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);          /* Core where the task should run */

  buzzer(IO_16, 200);
}

void loop()
{
  read_BT();

  if (isConnected)
  {
    digitalWrite(LED_ESP, HIGH);
    if(buzzer_connect){
      buzzer(IO_16, 100);
      buzzer_connect = false;
    };
    Xem_data_read_espnow();

    if (START)
    {
      // Serial.printf("DRI: %d, SPEED: %d, DIR_LR: %d, SPEED_LR: %d \n", DIR, SPEED, DIR_LR, SPEED_LR);
      control_motor(); // Control kit sumo robot

      while (BT1)
      {
        read_BT();

        digitalWrite(LED_USER, !digitalRead(LED_USER));
        delay(100);
        if (buzzer_warning)
        {
          buzzer_warning = 0;
          buzzer(IO_16, 100);
          delay(100);
          buzzer(IO_16, 100);
        }

        if (read_data.left_JoyX_value < 100 && read_data.left_JoyY_value < 100 && read_data.right_JoyX_value > 4000 && read_data.right_JoyY_value < 100 && !pass1 && !pass2)
        {
          pass1 = 1;
          buzzer(IO_16, 300);
        }
        else if (read_data.left_JoyX_value > 4000 && read_data.left_JoyY_value < 100 && read_data.right_JoyX_value < 100 && read_data.right_JoyY_value < 100 && pass1 && !pass2)
        {
          pass2 = 1;
          buzzer(IO_16, 300);
        }

        if (pass1 && pass2)
        {
          delay(2000);
          digitalWrite(IO_17, HIGH);
        }

        Serial.printf("LEFT: RVX: %d, RVY: %d",
                      read_data.left_JoyX_value,
                      read_data.left_JoyY_value);
        Serial.printf("----- RIGH: RVX: %d, RVY: %d \n",
                      read_data.right_JoyX_value,
                      read_data.right_JoyY_value);
      }
      buzzer_warning = 1;
      pass1 = 0;
      pass2 = 0;
      digitalWrite(LED_USER, HIGH);
    }
    else
    {
      digitalWrite(LED_USER, !digitalRead(LED_USER));
      delay(100);
      static uint8_t start1 = 0, start2 = 0;
      if (read_data.left_JoyY_value > 4000 && !start1 && !start2)
      {
        start1 = 1;
        digitalWrite(IO_16, LOW);
        delay(300);
        digitalWrite(IO_16, HIGH);
      }
      else if (read_data.left_JoyY_value < 100 && start1 && !start2)
      {
        start2 = 1;
        digitalWrite(IO_16, LOW);
        delay(300);
        digitalWrite(IO_16, HIGH);
      }
      if (start1 && start2)
      {
        START = 1;
        digitalWrite(LED_USER, HIGH);
        while(read_data.left_JoyY_value < 1800){
          static uint16_t time_buzzer = 0;
          read_BT();
          time_buzzer++;
          if(time_buzzer > 10){
            buzzer(IO_16, 50);
            time_buzzer = 0;
          }
          delay(10);
        }
      }
    }
  }
  else
  {
    digitalWrite(LED_ESP, LOW);
    memset(&read_data, 0, sizeof(read_data));
    CoEMaker.Motor_Stop_Short_brake();
    if(!buzzer_connect) buzzer(IO_16, 100);
    buzzer_connect = true;
  }
  delay(10);
}

void OnDataRecv()
{
  // Kiem tra SW tren joystick left
  if (read_data.left_JoySW_value)
  {
    BT_Left ^= 1;
  }
  if (read_data.left_Button_A_value)
  {
    BT1 ^= 1;
  }
  if (read_data.left_Button_B_value)
  {
    BT2 ^= 1;
  }

  // Kiem tra SW tren joystick right
  if (read_data.right_JoySW_value)
  {
    BT_Right ^= 1;
  }
  if (read_data.right_Button_A_value)
  {
    BT3 ^= 1;
  }
  if (read_data.right_Button_B_value)
  {
    BT4 ^= 1;
  }

  // Kiem tra truc X tren JoysTick left cho sumo bobot chay thuan nghich
  int readY_left = read_data.left_JoyY_value;
  if (readY_left > 1900)
  {
    DIR = 1;
    SPEED = map(readY_left, 1900, 4090, 0, 10);
  }
  else if (readY_left < 1800)
  {
    DIR = 0;
    SPEED = map(readY_left, 1800, 0, 0, 10);
  }
  else
  {
    DIR = -1;
    SPEED = 0;
  }

  // Kiem tra truc Y tren JoysTick right cho sumo bobot re trai hoac phai
  int readX_right = read_data.right_JoyX_value;
  if (readX_right > 1900)
  {
    DIR_LR = 0;
    SPEED_LR = map(readX_right, 1900, 4090, 0, 10);
  }
  else if (readX_right < 1800)
  {
    DIR_LR = 1;
    SPEED_LR = map(readX_right, 1800, 0, 0, 10);
  }
  else
  {
    DIR_LR = -1;
    SPEED_LR = 0;
  }

  // Kiem tra truc Y tren JoysTick right cho sumo bobot
  int readY_right = read_data.right_JoyY_value;
  if (readY_right > 1900)
  {
    DIR_xoay = 1;
    xoay_tron = map(readY_right, 1900, 4090, 0, 10);
  }
  else if (readY_right < 1700)
  {
    DIR_xoay = 0;
    xoay_tron = map(readY_right, 1700, 0, 0, 10);
  }
  else
  {
    DIR_xoay = -1;
    xoay_tron = 0;
  }
}
