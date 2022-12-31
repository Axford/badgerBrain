#ifndef OTA_MANAGER_
#define OTA_MANAGER_

#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "StatusDisplay.h"

class OTAManager {
  public:
    boolean isUpdating;
    OTAManager(AsyncEventSource * events, StatusDisplay * disp);

    void init(String hostname);
    void loop();
  private:
    StatusDisplay * _statusDisplay;
    AsyncEventSource *_events;
};

#endif
