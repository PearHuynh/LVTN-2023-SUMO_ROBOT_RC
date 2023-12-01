#include <Arduino.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include <SimpleKalmanFilter.h>

#define left_JoyX 34
#define left_JoyY 35
#define left_JoySW 13
#define left_Button_A 19
#define left_Button_B 21

#define right_JoyX 39
#define right_JoyY 36
#define right_JoySW 18
#define right_Button_A 22
#define right_Button_B 23

#define LED_ESP 2
#define LED_USER 27
#define DIP_SW 32

uint8_t state_send = 0;

/*--------------------------------------------------------------------//
  * Kalman fillter for ADC
//--------------------------------------------------------------------*/
SimpleKalmanFilter Kalman_left_RVX(2, 2, 0.01);
SimpleKalmanFilter Kalman_left_RVY(2, 2, 0.01);
SimpleKalmanFilter Kalman_right_RVX(2, 2, 0.01);
SimpleKalmanFilter Kalman_right_RVY(2, 2, 0.01);

/*--------------------------------------------------------------------//
  * State read value JoysTick
//--------------------------------------------------------------------*/
int right_JoyX_state;
int right_JoyY_state;
int left_JoyX_state;
int left_JoyY_state;

/*--------------------------------------------------------------------//
  *  REPLACE WITH YOUR RECEIVER MAC Address
  * Address Sender =
//--------------------------------------------------------------------*/
// Cau hinh bluetooth
BluetoothSerial SerialBT_Sender;
uint8_t broadcastAddress[6] = {0x24, 0xDC, 0xC3, 0xD0, 0x19, 0x82}; // esp32_flipper = 24:DC:C3:D0:19:82

char buffer_send[250];
StaticJsonDocument<250> json_sender;

/*--------------------------------------------------------------------//
  *  Structure to send data
  *  Must match the receiver structure
//--------------------------------------------------------------------*/
typedef struct struct_message
{
  int left_JoyX_value;
  int left_JoyY_value;
  uint8_t left_JoySW_value;
  uint8_t left_Button_A_value;
  uint8_t left_Button_B_value;

  int right_JoyX_value;
  int right_JoyY_value;
  uint8_t right_JoySW_value;
  uint8_t right_Button_A_value;
  uint8_t right_Button_B_value;

  uint8_t status = 0;
} struct_message;

// Create a struct_message called myData
struct_message senderData;

void LED_Warning(void *parameter)
{
  while (1)
  {
    if (!SerialBT_Sender.connected())
    {
      digitalWrite(LED_USER, !(digitalRead(LED_USER)));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup_stask()
{
  xTaskCreate(LED_Warning, "LED_Warning", 1000, NULL, 1, NULL);
}

int read_DIP_SW();
int KeyScan();
void Send_KEY();
void create_json();

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(right_JoyX, INPUT);
  pinMode(right_JoyY, INPUT);
  pinMode(right_JoySW, INPUT_PULLUP);
  pinMode(right_Button_A, INPUT_PULLUP);
  pinMode(right_Button_B, INPUT_PULLUP);

  pinMode(left_JoyX, INPUT);
  pinMode(left_JoyY, INPUT);
  pinMode(left_JoySW, INPUT_PULLUP);
  pinMode(left_Button_A, INPUT_PULLUP);
  pinMode(left_Button_B, INPUT_PULLUP);

  pinMode(LED_ESP, OUTPUT);
  pinMode(LED_USER, OUTPUT);
  pinMode(DIP_SW, INPUT);

  // uint16_t read_dip_sw = read_DIP_SW();
  // Serial.println("Read DIP SW = " + String(read_dip_sw));

  // if (read_dip_sw == 0)
  // {
  //   for (int i = 0; i < sizeof(broadcastAddress); ++i)
  //   {
  //     broadcastAddress[i] = broadcastAddress1[i];
  //   }
  // }
  // else if (read_dip_sw == 2) {
  //   for (int i = 0; i < sizeof(broadcastAddress); ++i) {
  //     broadcastAddress[i] = broadcastAddress2[i];
  //   }
  // }

  SerialBT_Sender.begin("CONTROLLER_ESP32", true);
  Serial.printf("The device started in master mode, make sure slave BT device is on!\n");
  while (!SerialBT_Sender.connect(broadcastAddress))
  {
  }
  Serial.println("Connected Successfully!");
  digitalWrite(LED_USER, HIGH);

  setup_stask();

  // senderData.left_JoyX_value = 1234;
  // senderData.left_JoyY_value = 5678;
  // senderData.left_JoySW_value = 1;
  // senderData.left_Button_A_value = 2;
  // senderData.left_Button_B_value = 3;

  // senderData.right_JoyX_value = 8765;
  // senderData.right_JoyY_value = 4321;
  // senderData.right_JoySW_value = 4;
  // senderData.right_Button_A_value = 5;
  // senderData.right_Button_B_value = 6;
}

void loop()
{
  if (!SerialBT_Sender.connected())
  {
    Serial.println("Bluetooth is not connect.");
    while (!SerialBT_Sender.connect(broadcastAddress))
    {
    }
    digitalWrite(LED_ESP, HIGH);
    Serial.println("Connected Successfully!");
  }

  // /*--------------------------------------------------------------------//
  // * Read ADC VRX and VRY JoysTick left
  //--------------------------------------------------------------------*/
  left_JoyX_state = analogRead(left_JoyX);
  senderData.left_JoyX_value = Kalman_left_RVX.updateEstimate(left_JoyX_state);

  left_JoyY_state = analogRead(left_JoyY);
  senderData.left_JoyY_value = Kalman_left_RVY.updateEstimate(left_JoyY_state);

  /*--------------------------------------------------------------------//
  * Read ADC VRX and VRY JoysTick right
  //--------------------------------------------------------------------*/
  right_JoyX_state = analogRead(right_JoyX);
  senderData.right_JoyX_value = Kalman_right_RVX.updateEstimate(right_JoyX_state);

  right_JoyY_state = analogRead(right_JoyY);
  senderData.right_JoyY_value = Kalman_right_RVY.updateEstimate(right_JoyY_state);

  /*--------------------------------------------------------------------//
  * Read KEY A & B & SW joysTick left and right
  //--------------------------------------------------------------------*/
  Send_KEY();

  /*--------------------------------------------------------------------//
  * Serial
  //--------------------------------------------------------------------*/

  create_json();

  // Serial.printf("LEFT: RVX: %d, RVY: %d, SM: %d, BTA: %d, BTB: %d --- ",
  //               senderData.left_JoyX_value,
  //               senderData.left_JoyY_value,
  //               senderData.left_JoySW_value,
  //               senderData.left_Button_A_value,
  //               senderData.left_Button_B_value);

  // Serial.printf("RIGH: RVX: %d, RVY: %d, SM: %d, BTA: %d, BTB: %d \n",
  //               senderData.right_JoyX_value,
  //               senderData.right_JoyY_value,
  //               senderData.right_JoySW_value,
  //               senderData.right_Button_A_value,
  //               senderData.right_Button_B_value);

  // Serial.println(read_DIP_SW());

  delay(20);
}

void create_json()
{
  json_sender.clear();

  json_sender["LX"] = senderData.left_JoyX_value;
  json_sender["LY"] = senderData.left_JoyY_value;
  json_sender["LW"] = senderData.left_JoySW_value;
  json_sender["LA"] = senderData.left_Button_A_value;
  json_sender["LB"] = senderData.left_Button_B_value;

  json_sender["RX"] = senderData.right_JoyX_value;
  json_sender["RY"] = senderData.right_JoyY_value;
  json_sender["RW"] = senderData.right_JoySW_value;
  json_sender["RA"] = senderData.right_Button_A_value;
  json_sender["RB"] = senderData.right_Button_B_value;

  senderData.status ^= 1;
  json_sender["ST"] = senderData.status;

  serializeJson(json_sender, buffer_send);
  Serial.println("Send: " + String(buffer_send));
  SerialBT_Sender.println(buffer_send);
}

/*--------------------------------------------------------------------//
  * read_DIP_SW
  * Doc chon dia chi can gui den de giao tiep.
//--------------------------------------------------------------------*/
int read_DIP_SW()
{
  int read_adc = analogRead(32);
  if (read_adc > 3580)
    return 7;
  if (read_adc > 3480)
    return 6;
  if (read_adc > 3280)
    return 5;
  if (read_adc > 3150)
    return 4;
  if (read_adc > 2850)
    return 3;
  if (read_adc > 2550)
    return 2;
  if (read_adc > 1750)
    return 1;
  return 0;
}

/*--------------------------------------------------------------------//
  * KeyScan
  * Quet kiem tra cac phim xem phim nao duoc an va tra ve gia tri tuong ung.
//--------------------------------------------------------------------*/
int KeyScan()
{
  static uint8_t keyBuf[3] = {0, 0, 0};
  int kq = 0; // Khong co an phim key = -1
  int key = 0;

  if (digitalRead(left_JoySW) == 0)
    key = 1;
  else if (digitalRead(left_Button_A) == 0)
    key = 2;
  else if (digitalRead(left_Button_B) == 0)
    key = 3;
  else if (digitalRead(right_JoySW) == 0)
    key = 4;
  else if (digitalRead(right_Button_A) == 0)
    key = 5;
  else if (digitalRead(right_Button_B) == 0)
    key = 6;

  keyBuf[0] = keyBuf[1];
  keyBuf[1] = keyBuf[2];
  keyBuf[2] = key; // Nha phim key = 0

  if ((keyBuf[0] == keyBuf[1]) && (keyBuf[2] == 0))
    kq = keyBuf[0];

  return kq;
}

/*--------------------------------------------------------------------//
  * Send_KEY
  * Quet kiem tra cac phim xem phim nao duoc an va luu vao bien Stuct tuong ung.
//--------------------------------------------------------------------*/
void Send_KEY()
{
  switch (KeyScan())
  {
  case 1:
    senderData.left_JoySW_value = 1;
    break;
  case 2:
    senderData.left_Button_A_value = 1;
    break;
  case 3:
    senderData.left_Button_B_value = 1;
    break;
  case 4:
    senderData.right_JoySW_value = 1;
    break;
  case 5:
    senderData.right_Button_A_value = 1;
    break;
  case 6:
    senderData.right_Button_B_value = 1;
    break;
  case 0:
    senderData.left_JoySW_value = 0;
    senderData.left_Button_A_value = 0;
    senderData.left_Button_B_value = 0;
    senderData.right_JoySW_value = 0;
    senderData.right_Button_A_value = 0;
    senderData.right_Button_B_value = 0;
    break;
  }
}
