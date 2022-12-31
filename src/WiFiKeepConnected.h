#ifndef WIFI_KEEP_CONNECTED_
#define WIFI_KEEP_CONNECTED_

#include "Arduino.h"

#define WIFI_TIMEOUT_MS       3000
#define WIFI_RECOVER_TIME_MS  10000

class WiFiKeepConnected {
public:

  WiFiKeepConnected();

  void start(const char * ssid, const char * pwd);

private:
  const char * _ssid;
  const char * _pwd;
  int _taskCore;
  int _taskPriority;
  TaskHandle_t _Task1;
  unsigned long _connectionCounter;

  static void keepWiFiAlive(void *pvParameters);
};


#endif
