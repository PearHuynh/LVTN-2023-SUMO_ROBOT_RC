#include "wifi_connect.h"

// Chương trình tạo new task--------------------------------------------------------------------------
void WIFI_CONNECT::wifi_begin(const char *ssid, const char *pass)
{
    this->SSID = ssid;
    this->PASS = pass;

    WiFi.begin(SSID, PASS);
    Serial.println("----------------------Connecting to Wifi " + String(SSID) + " ------------------------");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(".");
        delay(500);
    }
    Serial.print("----------------------Connect with IP: ");
    Serial.println(WiFi.localIP());

    WiFi.setAutoReconnect(true);
}
bool WIFI_CONNECT::led_state_wifi()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;
    else
        return false;
}

WIFI_CONNECT _wifi;
