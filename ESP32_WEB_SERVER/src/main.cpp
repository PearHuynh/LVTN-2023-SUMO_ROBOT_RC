#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>
#include "wifi_connect.h"
#include "WebServerESP32.h"
#include <SPIFFS.h>

WebServer webServer(80);
TaskHandle_t Task1;

#define LED_ESP 2
#define LED_USE 12

// OUTPUT MOTOR LEFT
#define PWM_L_PIN 25
#define DIR1_L_PIN 26
#define DIR2_L_PIN 27

// OUTPUT MOTOR RIGHT
#define PWM_R_PIN 22
#define DIR1_R_PIN 23
#define DIR2_R_PIN 21

typedef struct struct_message
{
  uint8_t setup_status = 0;
  uint8_t in_out = 0;
  uint8_t status = 0;
  String state_set = "IN";
} struct_message;

struct_message _gpio_13, _gpio_14,
    _gpio_16, _gpio_17,
    _gpio_18, _gpio_19,
    _gpio_32, _gpio_33,
    _gpio_34, _gpio_35, _gpio_36,
    _gpio_39;

typedef struct struct_control
{
  uint8_t mode_speed = 0;
  int8_t dir_control_left = 1;
  int8_t dir_control_right = 1;
  uint16_t speed_left = 0;
  uint16_t speed_right = 0;
  uint8_t state_set_mode = 0;
  String state_set_dir = "";
  uint8_t start = 0;
  uint16_t delay_start = 0;
} struct_control;

struct_control control;

StaticJsonDocument<600> json_doc;
StaticJsonDocument<250> jsonDocument;

int delay_control = 0;

// PWM Output
const uint16_t freq_15kHz = 15000;
const uint8_t channal_PWM_L = 8; // Channal 8 (0 to 16 channal)
const uint8_t channal_PWM_R = 9; // Channal 9 (0 to 16 channal)
const uint8_t resolution = 10;

/*------GPIO_pin---0(13)---1(14)---2(16)---3(17)---4(18)---5(19)---6(32)---7(33)---8(34)---9(35)--10(36)--11(39)*/
uint8_t GPIO_pin[12] = {13, 14, 16, 17, 18, 19, 32, 33, 34, 35, 36, 39};

/*------GPIO_struct---0(13)---1(14)---2(16)---3(17)---4(18)---5(19)---6(32)---7(33)*/
struct_message GPIO_struct[] = {
    _gpio_13, _gpio_14,
    _gpio_16, _gpio_17,
    _gpio_18, _gpio_19,
    _gpio_32, _gpio_33,
    _gpio_34, _gpio_35, _gpio_36,
    _gpio_39};

void Task1code_0(void *parameter);
void mainpase();
void gpio_status();
void handle_set_mode();
void handle_set_out();
String Read_status_GPIO();
void handle_mode_speed();
void handle_dir_left();
void handle_dir_right();
void handle_speed_left();
void handle_speed_right();
void handle_start_motor();
void fc_control_speed();
void fc_setting_speed();

/*------------------------------------STATUS WIFI--------------------------------------*/
void status_wifi(void *parameter)
{
  TickType_t last_tick_count = xTaskGetTickCount();
  TickType_t period_200 = pdMS_TO_TICKS(200);
  TickType_t period_1000 = pdMS_TO_TICKS(1000);
  bool st = 0;
  while (1)
  {
    if (_wifi.led_state_wifi())
    {
      st = !st;
      digitalWrite(LED_ESP, st);
      vTaskDelayUntil(&last_tick_count, period_1000);
    }
    else
    {
      st = !st;
      digitalWrite(LED_ESP, st);
      vTaskDelayUntil(&last_tick_count, period_200);
    }
  }
}

void setup_control_motor()
{
  // Khai bao PWM
  ledcSetup(channal_PWM_L, freq_15kHz, resolution);
  ledcSetup(channal_PWM_R, freq_15kHz, resolution);
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_L_PIN, channal_PWM_L);
  ledcAttachPin(PWM_R_PIN, channal_PWM_R);

  // OUTPUT Driver
  pinMode(DIR1_L_PIN, OUTPUT);
  pinMode(DIR2_L_PIN, OUTPUT);
  pinMode(DIR1_R_PIN, OUTPUT);
  pinMode(DIR2_R_PIN, OUTPUT);

  // Set trang thai ban dau.
  digitalWrite(DIR1_L_PIN, LOW);
  digitalWrite(DIR2_L_PIN, LOW);
  digitalWrite(DIR1_R_PIN, LOW);
  digitalWrite(DIR2_R_PIN, LOW);
}

/*------------------------------------SETUP--------------------------------------------*/
void setup()
{
  Serial.begin(115200);

  Serial.print("This main runs on Core: ");
  Serial.println(xPortGetCoreID());

  // Setup start core 0
  xTaskCreatePinnedToCore(
      Task1code_0, /* Function to implement the task */
      "Task1",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &Task1,      /* Task handle. */
      0);          /* Core where the task should run */

  setup_control_motor();

  for (int pin = 0; pin <= 11; pin++)
  {
    pinMode(GPIO_pin[pin], INPUT);
  }

  pinMode(LED_ESP, OUTPUT);

  // _wifi.wifi_begin("TP-Link_Pear", "03102003");
  _wifi.wifi_begin("Pear", "03102003");
  xTaskCreate(status_wifi, "status_wifi", 1023, NULL, 1, NULL);

  webServer.on("/", mainpase);
  // Set web for box left
  webServer.on("/gpio_status", gpio_status);
  webServer.on("/receive_data", HTTP_POST, handle_set_mode);
  webServer.on("/receive_status", HTTP_POST, handle_set_out);
  // Set web for box right
  webServer.on("/mode_speed", HTTP_POST, handle_mode_speed);
  webServer.on("/dir_left", HTTP_POST, handle_dir_left);
  webServer.on("/dir_right", HTTP_POST, handle_dir_right);
  webServer.on("/receiver_speed_left", HTTP_POST, handle_speed_left);
  webServer.on("/receiver_speed_right", HTTP_POST, handle_speed_right);
  webServer.on("/start_motor", HTTP_POST, handle_start_motor);

  webServer.begin();

  int delay_control = millis();
}

/*------------------------------------LOOP--------------------------------------------*/
void loop()
{
  webServer.handleClient();

  if (millis() - delay_control > 10)
  {
    if (control.state_set_mode == 0)
    {
      fc_control_speed();
    }
    delay_control = millis();
  }
}

/*------------------------------------LOOP--------------------------------------------*/
void Task1code_0(void *parameter)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  pinMode(LED_USE, OUTPUT);
  int delay_set = millis();
  while (1)
  {
    if (millis() - delay_set > 10)
    {
      if (control.state_set_mode == 1)
      {
        fc_setting_speed();
      }

      delay_set = millis();
    }
    digitalWrite(LED_USE, !digitalRead(LED_USE));
    // Serial.println("dir_left: " + String(control.dir_control_left) + " --- dir_right: " + String(control.dir_control_right));
    delay(1000);
  }
}

/*------------------------------------ADD MAIN PAGE HTML------------------------------*/
void mainpase()
{
  String webserver = FPSTR(Main_Page);
  webServer.send(200, "text/html", webserver);
}

/*------------------------------------GPIO STATUS-------------------------------------*/
void gpio_status()
{
  // serializeJson(json_doc, buffer);
  String buff_json = Read_status_GPIO();
  // Serial.println(buff_json);
  webServer.send(200, "application/json", buff_json);
}

/*------------------------------------READ STATUS GPIO----------------------------------*/
String Read_status_GPIO()
{
  String buffer;
  const int numGpio = 12;
  const int analogGpioIndex = 11;
  json_doc.clear();

  for (int i = 0; i < numGpio; i++)
  {
    String digitalKey = "ST" + String(GPIO_pin[i]);
    String stateKey = "S" + String(GPIO_pin[i]);
    if (i <= 7)
    {
      json_doc[digitalKey] = digitalRead(GPIO_pin[i]);
      json_doc[stateKey] = GPIO_struct[i].state_set;
    }
    else
    {
      json_doc[digitalKey] = digitalRead(GPIO_pin[i]);
    }
    if (i == analogGpioIndex)
    {
      json_doc[digitalKey] = analogRead(GPIO_pin[i]);
    }
  }
  json_doc["CS"] = control.state_set_mode;
  serializeJson(json_doc, buffer);
  // Serial.println(buffer);
  return buffer;
}

/*------------------------------------HANDLE SET MODE----------------------------------*/
void handle_set_mode()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    Serial.println(body);
    // Đọc các giá trị từ server
    int ioNumber = jsonDocument["IO"];
    for (int gpio = 0; gpio <= 7; gpio++)
    {
      if (ioNumber == GPIO_pin[gpio])
      {
        GPIO_struct[gpio].setup_status = 1;
        Serial.println("Select " + String(GPIO_pin[gpio]) + ": " + String(GPIO_struct[gpio].in_out));
        if (jsonDocument["ST"])
        {
          if (GPIO_struct[gpio].setup_status)
          {
            GPIO_struct[gpio].setup_status = 0;
            pinMode(GPIO_pin[gpio], OUTPUT);
            digitalWrite(GPIO_pin[gpio], LOW);
            Serial.println("Set gpio " + String(GPIO_pin[gpio]) + ": OUTPUT");
            GPIO_struct[gpio].state_set = "OUT";
            delay(10);
          }
        }
        else
        {
          if (GPIO_struct[gpio].setup_status)
          {
            GPIO_struct[gpio].setup_status = 0;
            pinMode(GPIO_pin[gpio], INPUT);
            Serial.println("Set gpio " + String(GPIO_pin[gpio]) + ": INPUT");
            GPIO_struct[gpio].state_set = "IN";
            delay(10);
          }
        }
        break;
      }
    }
  }
  // Phản hồi về OK về client
  String buff_mode = Read_status_GPIO();
  webServer.send(200, "application/json", buff_mode);
}

/*------------------------------------HANDLE SET OUT----------------------------------*/
void handle_set_out()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String set_gpio = webServer.arg("plain");
    deserializeJson(jsonDocument, set_gpio);
    // In ra màn hình để kiểm tra
    Serial.println(set_gpio);
    // Đọc các giá trị từ server
    int ioNumber = jsonDocument["IO"];

    for (int gpio = 0; gpio <= 7; gpio++)
    {
      if (ioNumber == GPIO_pin[gpio])
      {
        GPIO_struct[gpio].status = jsonDocument["ST"];
        if (GPIO_struct[gpio].status)
        {
          digitalWrite(GPIO_pin[gpio], HIGH);
        }
        else
        {
          digitalWrite(GPIO_pin[gpio], LOW);
        }
        break;
      }
    }
  }
  // Phản hồi về OK về client
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE MODE SPEED----------------------------------*/
void handle_mode_speed()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    Serial.println(body);
    control.mode_speed = jsonDocument["MD"];
    if (control.mode_speed)
    {
      Serial.println("Set MODE SPEED: Setting Speed");
      while (control.state_set_mode == 0)
      {
        control.state_set_mode = 1;
      }
    }
    else
    {
      Serial.println("Set MODE SPEED: Control Speed");
      while (control.state_set_mode == 1)
      {
        control.state_set_mode = 0;
      }
    }
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE DIR MOTOR----------------------------------*/
void handle_dir_left() // Set dir motor left
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    control.dir_control_left = jsonDocument["ML"];
    Serial.println(body);
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}
void handle_dir_right() // Set dir motor right
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    control.dir_control_right = jsonDocument["MR"];
    Serial.println(body);
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE SPEED LEFT----------------------------------*/
void handle_speed_left()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    control.speed_left = jsonDocument["SPL"];
    Serial.println(body);
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE SPEED RIGHT----------------------------------*/
void handle_speed_right()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    control.speed_right = jsonDocument["SPR"];
    Serial.println(body);
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE START MOTOR----------------------------------*/
void handle_start_motor()
{
  if (webServer.hasArg("plain") == true)
  {
    // Đọc nội dung yêu cầu post
    String body = webServer.arg("plain");
    Serial.println(body);
    deserializeJson(jsonDocument, body);
    // In ra màn hình để kiểm tra
    control.delay_start = jsonDocument["DL"];
    control.start = 1;
  }
  String buff_sta = Read_status_GPIO();
  webServer.send(200, "application/json", buff_sta);
}

/*------------------------------------HANDLE SPEED RIGHT----------------------------------*/
void fc_control_speed()
{
  if (control.dir_control_left == 1)
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, HIGH);
    digitalWrite(DIR2_L_PIN, LOW);
    ledcWrite(channal_PWM_L, control.speed_left);
  }
  else if (control.dir_control_left == 0)
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, LOW);
    digitalWrite(DIR2_L_PIN, HIGH);
    ledcWrite(channal_PWM_L, control.speed_left);
  }
  else
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, LOW);
    digitalWrite(DIR2_L_PIN, LOW);
  }

  if (control.dir_control_right == 1)
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, HIGH);
    digitalWrite(DIR2_R_PIN, LOW);
    ledcWrite(channal_PWM_R, control.speed_right);
  }
  else if (control.dir_control_right == 0)
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, LOW);
    digitalWrite(DIR2_R_PIN, HIGH);
    ledcWrite(channal_PWM_R, control.speed_right);
  }
  else
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, LOW);
    digitalWrite(DIR2_R_PIN, LOW);
  }
  // Serial.println("Control speed left: " + String(control.speed_left) + " --- Control speed right: " + String(control.speed_right));
  // Serial.println("DIR_SPEED_left: "+String(control.dir_control_left)+
  //             " --- SPEED_left: "+String(control.speed_left)+
  //             " --- DIR_SPEED_right: "+String(control.dir_control_right)+
  //             " --- SPEED_right: "+String(control.speed_right));
}

void fc_setting_speed()
{
  if (control.dir_control_left == 1)
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, HIGH);
    digitalWrite(DIR2_L_PIN, LOW);
  }
  else if (control.dir_control_left == 0)
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, LOW);
    digitalWrite(DIR2_L_PIN, HIGH);
  }
  else
  {
    ledcWrite(channal_PWM_L, 0);
    digitalWrite(DIR1_L_PIN, LOW);
    digitalWrite(DIR2_L_PIN, LOW);
  }

  if (control.dir_control_right == 1)
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, HIGH);
    digitalWrite(DIR2_R_PIN, LOW);
  }
  else if (control.dir_control_right == 0)
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, LOW);
    digitalWrite(DIR2_R_PIN, HIGH);
  }
  else
  {
    ledcWrite(channal_PWM_R, 0);
    digitalWrite(DIR1_R_PIN, LOW);
    digitalWrite(DIR2_R_PIN, LOW);
  }

  if (control.start == 1)
  {
    control.start = 0;
    ledcWrite(channal_PWM_L, control.speed_left);
    ledcWrite(channal_PWM_R, control.speed_right);
    delay(control.delay_start);
  }
}
