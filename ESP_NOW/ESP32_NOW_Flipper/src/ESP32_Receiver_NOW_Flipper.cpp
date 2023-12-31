#include <Arduino.h>
#include "Read_NOW.h"
#include <WiFi.h>
#include <esp_now.h>

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

uint8_t start_cam = 0;

/*------------------------------------LOOP--------------------------------------------*/
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

void setup()
{
  // Set buzzer
  pinMode(IO_16, OUTPUT);
  digitalWrite(IO_16, HIGH);

  // Set on kit
  on_kit();

  // Initialize Serial Monitor
  Serial.begin(115200);

  Serial.println("");
  Serial.print("Main running on core ");
  Serial.println(xPortGetCoreID());

  // Setup start core 0
  xTaskCreatePinnedToCore(
      Task1code_0, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      1,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);          /* Core where the task should run */

  // Print some system and software info to serial monitor
  delay(1000);
  Serial.println("");
  Serial.println("------------------------------------------------------------------------------------------------");
  Serial.println("2023 Design and Manufacture of Sumo Robot. Control Engineering and Automation Thesis.");
  // Serial.printf("https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("XTAL Frequency: %i MHz, CPU Clock: %i MHz, APB Bus Clock: %i Hz\n", getXtalFrequencyMhz(), getCpuFrequencyMhz(), getApbFrequency());
  Serial.printf("Internal RAM size: %i Byte, Free: %i Byte\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("WiFi MAC address: %s\n", WiFi.macAddress().c_str());
  Serial.println("------------------------------------------------------------------------------------------------");
  Serial.println("");

  // Set kit sumo robot
  CoEMaker.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  /*--------------------------------------------------------------------//
  * Once ESPNow is successfully Init, we will register for recv CB to
  * get recv packer info
  //--------------------------------------------------------------------*/
  esp_now_register_recv_cb(OnDataRecv);

  // warning on kit
  buzzer(IO_16, 200);
}

void loop()
{
  Xem_data_read_espnow(); // Xem serial
  if (++State_connect > 5)
  {
    digitalWrite(LED_ESP, LOW);
    CoEMaker.Motor_Stop_Short_brake();
  }
  else
  {
    digitalWrite(LED_ESP, HIGH);
    if (START)
    {
      // Serial.printf("DRI: %d, SPEED: %d, DIR_LR: %d, SPEED_LR: %d \n", DIR, SPEED, DIR_LR, SPEED_LR);
      control_motor(); // Control kit sumo robot

      while (BT1)
      {
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
          buzzer(IO_16, 100);
          delay(1000);
        }
      }
    }
  }

  delay(10);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (memcmp(mac, broadcastAddress, sizeof(broadcastAddress)) == 0)
  {
    memcpy(&read_data, incomingData, sizeof(read_data));
    // Serial.print("Bytes received: ");
    // Serial.println(len);

    // Trang thai van con ket noi bien duoc reset 0 lien tuc
    State_connect = 0;

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

    // Kiem tra truc X tren JoysTick left cho sumo bobot chay tien lui
    int readY_left = read_data.left_JoyY_value;
    if (readY_left > 1900)
    {
      DIR = 1;
      SPEED = map(readY_left, 1900, 4090, 0, 1023);
    }
    else if (readY_left < 1800)
    {
      DIR = 0;
      SPEED = map(readY_left, 1800, 0, 0, 1023);
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
      SPEED_LR = map(readX_right, 1900, 4090, 0, 1023);
    }
    else if (readX_right < 1800)
    {
      DIR_LR = 1;
      SPEED_LR = map(readX_right, 1800, 0, 0, 1023);
    }
    else
    {
      DIR_LR = -1;
      SPEED_LR = 0;
    }

    // Kiem tra truc Y tren JoysTick right cho sumo bobot xoay tron
    int readY_right = read_data.right_JoyY_value;
    if (readY_right > 1900)
    {
      DIR_xoay = 1;
      xoay_tron = map(readY_right, 1900, 4090, 0, 1023);
    }
    else if (readY_right < 1700)
    {
      DIR_xoay = 0;
      xoay_tron = map(readY_right, 1700, 0, 0, 1023);
    }
    else
    {
      DIR_xoay = -1;
      xoay_tron = 0;
    }
  }
  else
  {
    // Serial.println("false address");
    digitalWrite(LED_USER, !digitalRead(LED_USER));
    delay(300);
  }
}
