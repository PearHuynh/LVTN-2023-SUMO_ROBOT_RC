#ifndef __WIFI_CONNECT_H
#define __WIFI_CONNECT_H

// Khai báo thư viện-----------------------------------------------------------------------------------
#include <WiFi.h>

// Khai báo chương trình con---------------------------------------------------------------------------
class WIFI_CONNECT
{
public:
  void wifi_begin(const char *ssid, const char *pass);
  bool led_state_wifi();

private:
  const char *SSID;
  const char *PASS;
};

extern WIFI_CONNECT _wifi;

#endif
