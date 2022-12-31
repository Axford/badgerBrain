#include "Arduino.h"
#include "WiFiKeepConnected.h"
#include "WiFi.h"
#include "esp_wifi.h"

WiFiKeepConnected::WiFiKeepConnected() {
  _taskCore = 1;
  _taskPriority = 1;
  _connectionCounter = 0;
}

void WiFiKeepConnected::start(const char * ssid, const char * pwd) {
  _ssid = ssid;
  _pwd = pwd;

  Serial.println("[WIFI] Starting connection task");

  WiFi.setSleep(false);

  xTaskCreatePinnedToCore(
    this->keepWiFiAlive,        //Function to implement the task
    "keepWiFiAlive",            //Name of the task
    5000,                   //Stack size in words
    this,                   //Task input parameter
    _taskPriority,           //Priority of the task
    &_Task1,                 //Task handle.
    _taskCore);              //Core where the task should run
}

void WiFiKeepConnected::keepWiFiAlive(void *pvParameters){
  WiFiKeepConnected *l_pThis = (WiFiKeepConnected *) pvParameters;

  for(;;){
    if(WiFi.status() == WL_CONNECTED){
      Serial.print("[WIFI] Connected: ");
      Serial.println(WiFi.localIP());
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }

    if (l_pThis->_connectionCounter > 0) {
      Serial.println("[WIFI] Off");
      WiFi.mode(WIFI_OFF);
    	vTaskDelay(1);
    }

    Serial.print("[WIFI] Connecting... SSID: ");
    Serial.print(l_pThis->_ssid);
    Serial.print(", pw: ");
    Serial.println(l_pThis->_pwd);

    WiFi.mode(WIFI_STA);
    //esp_wifi_set_ps (WIFI_PS_NONE);
    //Esp_wifi_set_ps (WIFI_PS_NONE);
    vTaskDelay(1);
    WiFi.begin(l_pThis->_ssid, l_pThis->_pwd);

    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't reached the timeout
    Serial.println("[WIFI] waiting for connection");
    while (WiFi.status() != WL_CONNECTED &&
    millis() < WIFI_TIMEOUT_MS + startAttemptTime){
      vTaskDelay(1);
    }

    // When we couldn't make a WiFi connection (or the timeout expired)
    // sleep for a while and then retry.
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("[WIFI] FAILED");
      vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
      continue;
    }

    l_pThis->_connectionCounter++;
  }
}
