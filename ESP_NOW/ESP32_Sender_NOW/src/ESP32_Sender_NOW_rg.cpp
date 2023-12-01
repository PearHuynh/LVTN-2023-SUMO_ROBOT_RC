#include <WiFi.h>
#include <esp_now.h>
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
SimpleKalmanFilter Kalman_DIPSW(2, 2, 0.01);
/*--------------------------------------------------------------------//
  * State read value JoysTick 
//--------------------------------------------------------------------*/
int right_JoyX_state;
int right_JoyY_state;
int left_JoyX_state;
int left_JoyY_state;

/*--------------------------------------------------------------------//
  *  REPLACE WITH YOUR RECEIVER MAC Address
  * Address Sender = C4:DE:E2:0E:CE:08
//--------------------------------------------------------------------*/
uint8_t broadcastAddress[] = { 0xB8, 0xD6, 0x1A, 0x42, 0xB5, 0x04 }; //Add: 02 = B8:D6:1A:42:B5:04
// uint8_t broadcastAddress1[] = { 0x24, 0xDC, 0xC3, 0XD0, 0x19, 0x80 }; //Add: 01 = 24:DC:C3:D0:19:80
// uint8_t broadcastAddress2[] = { 0xB8, 0xD6, 0x1A, 0x42, 0xB5, 0x04 }; //Add: 02 = B8:D6:1A:42:B5:04
// uint8_t broadcastAddress3[] = { 0xFC, 0xB4, 0x67, 0x51, 0x05, 0xD8 }; //Add: 03 = FC:B4:67:51:05:D8

/*--------------------------------------------------------------------//
  *  Structure to send data
  *  Must match the receiver structure
//--------------------------------------------------------------------*/
typedef struct struct_message {
  int left_JoyX_value;          //Read ADC X Joystick left
  int left_JoyY_value;          //Read ADC Y Joystick left
  uint8_t left_JoySW_value;     //Read Button Joystick left
  uint8_t left_Button_A_value;  //Read Button A left
  uint8_t left_Button_B_value;  //Read Button B left

  int right_JoyX_value;         //Read ADC X Joystick right
  int right_JoyY_value;         //Read ADC Y Joystick right
  uint8_t right_JoySW_value;    //Read Button Joystick right
  uint8_t right_Button_A_value; //Read Button A right
  uint8_t right_Button_B_value; //Read Button B right
} struct_message;

// Create a struct_message called senderData
struct_message senderData;

/*--------------------------------------------------------------------//
  * now peer info
//--------------------------------------------------------------------*/
esp_now_peer_info_t peerInfo;

int read_DIP_SW();
int KeyScan();
void Send_KEY();

/*--------------------------------------------------------------------//
  *  Callback when data is sent
//--------------------------------------------------------------------*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS && state_send) {
    digitalWrite(LED_USER, HIGH);
  } else {
    digitalWrite(LED_USER, LOW);
  }
}

void setup() {
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

  pinMode(LED_USER, OUTPUT);
  pinMode(DIP_SW, INPUT);

  /*--------------------------------------------------------------------//
  * Set device as a Wi-Fi Station
  //--------------------------------------------------------------------*/
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // uint8_t read_dip_sw = read_DIP_SW();
  // Serial.println("Read DIP SW = " +String(read_dip_sw));

  // if (read_dip_sw == 1) {
  //   for (int i = 0; i < sizeof(broadcastAddress); ++i) {
  //     broadcastAddress[i] = broadcastAddress1[i];
  //   }
  // } 
  // else if (read_dip_sw == 2) {
  //   for (int i = 0; i < sizeof(broadcastAddress); ++i) {
  //     broadcastAddress[i] = broadcastAddress2[i];
  //   }
  // }
  // else if (read_dip_sw == 3) {
  //   for (int i = 0; i < sizeof(broadcastAddress); ++i) {
  //     broadcastAddress[i] = broadcastAddress3[i];
  //   }
  // }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  /*--------------------------------------------------------------------//
  * Read ADC VRX and VRY JoysTick left
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

  /*--------------------------------------------------------------------//
  * Send message via ESP-NOW
  //--------------------------------------------------------------------*/
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&senderData, sizeof(senderData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    Serial.println("");
    state_send = 1;
  } else {
    Serial.println("Error sending the data");
    Serial.println("");
    state_send = 0;
  }
  delay(10);
}

/*--------------------------------------------------------------------//
  * read_DIP_SW
  * Doc chon dia chi can gui den de giao tiep.
//--------------------------------------------------------------------*/
int read_DIP_SW() {
  int read_adc;
  for (int i = 0; i < 10; i++) {
    read_adc = Kalman_DIPSW.updateEstimate(analogRead(32));
    // Serial.println("Chua loc" + String(read_adc));
  }
  // int read_adc = analogRead(32);
  if (read_adc > 3580) return 7;
  if (read_adc > 3480) return 6;
  if (read_adc > 3280) return 5;
  if (read_adc > 3150) return 4;
  if (read_adc > 2850) return 3;
  if (read_adc > 2550) return 2;
  if (read_adc > 1750) return 1;
  return 0;
}

/*--------------------------------------------------------------------//
  * KeyScan
  * Quet kiem tra cac phim xem phim nao duoc an va tra ve gia tri tuong ung.
//--------------------------------------------------------------------*/
int KeyScan() {
  static uint8_t keyBuf[3] = { 0, 0, 0 };
  int kq = 0;  //Khong co an phim key = -1
  int key = 0;

  if (digitalRead(left_JoySW) == 0) key = 1;
  else if (digitalRead(left_Button_A) == 0) key = 2;
  else if (digitalRead(left_Button_B) == 0) key = 3;
  else if (digitalRead(right_JoySW) == 0) key = 4;
  else if (digitalRead(right_Button_A) == 0) key = 5;
  else if (digitalRead(right_Button_B) == 0) key = 6;

  keyBuf[0] = keyBuf[1];
  keyBuf[1] = keyBuf[2];
  keyBuf[2] = key;  //Nha phim key = 0

  if ((keyBuf[0] == keyBuf[1]) && (keyBuf[2] == 0))
    kq = keyBuf[0];

  return kq;
}

/*--------------------------------------------------------------------//
  * Send_KEY
  * Quet kiem tra cac phim xem phim nao duoc an va luu vao bien Stuct tuong ung.
//--------------------------------------------------------------------*/
void Send_KEY() {
  switch (KeyScan()) {
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
