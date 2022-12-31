/*

badgerBrain

to run on ESP32 Dev Module

*/

#include "Arduino.h"
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include "HardwareSerial.h"
#include "WiFiKeepConnected.h"
#include "OTAManager.h"
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFSEditor.h>
#include <Geometry.h>
#include <Wire.h>
#include <DynamixelMotor.h>
#include <esp_task_wdt.h>

#include "StatusDisplay.h"
#include "LegController.h"
#include "BadgerController.h"
#include "SoftwareSerial.h"  // the ESP32 version

#define I2C_SDA  4
#define I2C_SCL  15

#define TRISTATE_DIR_PIN    5

HardwareDynamixelInterface servoInterface(Serial2, TRISTATE_DIR_PIN);

#define WIFI_NETWORK   "Badger"
#define WIFI_PASSWORD  "LouisVuitton"

uint8_t StatusPixelCount = 16;
StatusDisplay statusDisplay((uint8_t)23, StatusPixelCount);  // status display on pin 23, 16 leds

String hostname = "badger";

HardwareSerial MySerial(2);
WiFiKeepConnected wifiKeepConnected;

AsyncWebServer server(80);
AsyncEventSource events("/events");
OTAManager OTAMgr(&events, &statusDisplay);

BadgerController badger(servoInterface);

TaskHandle_t _updateTask;

int newMode;
boolean newModeRequested = false;

int newGaitLegsRaised;
boolean newGaitRequested = false;

unsigned long _updateCount;  // how may updates since start
unsigned long startTime;  // what time did setup complete

float fps;


String pointToStr(Point p) {
  return '[' + String(p(0)) + ',' + String(p(1)) + ',' + String(p(2)) + ']';
}


void updateStatus() {
  static uint8_t spinPos = 0;

  // update color
  if (WiFi.status() == WL_CONNECTED && badger.ready && !badger.error) {

    // show leg status
    RgbColor clr;
    uint8_t legIndex;
    for (uint8_t i=0; i<StatusPixelCount; i++) {

      // convert i to leg index
      if (i > 0 && i < 7) {
        legIndex = 3 - round(i / 2.0f);
      } else if (i > 8 && i < 15) {
        legIndex = 6 - round((i - 8 ) / 2.0f);
      } else {
        legIndex = 255;
      }

      if (legIndex < 6) {
        clr = HslColor(
          badger.legs[legIndex]->_jointAngles[0] / 3.0f + 0.5f,
          badger.legs[legIndex]->ready ? 1.0f : 0.0f,
          0.1f
        );
      } else {
        clr = HslColor(0,0,0);
      }

      statusDisplay.setPixel(i, clr);
    }

  } else {
    RgbColor clr;

    // waiting for connection spinner
    for (uint8_t i=0; i<StatusPixelCount; i++) {
      if (i == spinPos) {
        clr = HslColor(0,1,0.1f);
      } else {
        clr = HslColor(0,0,0);
      }
      statusDisplay.setPixel(i, clr);
    }


    spinPos++;
    if (spinPos > StatusPixelCount-1) spinPos = 0;
  }


  statusDisplay.show();
}


void updateTask(void *pvParameters) {
  esp_task_wdt_init(5,0);

  for(;;){
    // check OTA not in Progress
    if (!OTAMgr.isUpdating) {

      if (newModeRequested) {
        badger.setMode(newMode);
        newModeRequested = false;
      }

      if (newGaitRequested) {
          badger.setGaitLegsRaised(newGaitLegsRaised);
          newGaitRequested = false;
      }

      badger.update();

      updateStatus();
      _updateCount++;
    }

    yield();
    vTaskDelay(1);
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println(F("[] Starting..."));
  statusDisplay.init();
  statusDisplay.setBrightness(0.1);
  statusDisplay.bargraph(0.1);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  if(!SPIFFS.begin(true)){
    Serial.println("[] SPIFFS Mount Failed");
    delay(3000);
    // Should be formatted and working after a reboot
    ESP.restart();
  }

  /* Initialise wire */
  //Wire.begin(I2C_SDA, I2C_SCL);

  wifiKeepConnected.start(WIFI_NETWORK, WIFI_PASSWORD);

  // init servo interface
  servoInterface.begin(1000000);
  delay(100);

  // init badger
  Serial.println("[] Init badger: ");
  //if (!badger.init()) {
  //  Serial.println("[ERROR] Failed to init badger");
  //}

  statusDisplay.bargraph(0.3);

  Serial.print("[] Hostname: ");
  Serial.println(hostname);

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });
  server.addHandler(&events);

  OTAMgr.init(hostname);

  MDNS.addService("http","tcp",80);

  statusDisplay.bargraph(0.5);

  server.addHandler(new SPIFFSEditor(SPIFFS));

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");


  server.on("/config", HTTP_GET, [] (AsyncWebServerRequest *request) {

    // work out which leg
    AsyncWebParameter* p = request->getParam("leg");
    uint8_t legIndex = p->value().toInt();
    if (legIndex > 5) legIndex = 5;

    AsyncResponseStream *response = request->beginResponseStream("application/json");
    StaticJsonDocument<768> doc;
    badger.legs[legIndex]->serializeConfig(doc);
    serializeJson(doc, *response);
    request->send(response);
  });


  server.on("/legState", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");

    response->print("{\"legs\":[\n");

    for (uint8_t i=0; i<6; i++) {

      response->print("{");
      response->print("\"jointAngles\":[");
      for (uint8_t j=0; j<3; j++) {
        response->print(String(badger.legs[i]->_jointAngles[j] * 180 / PI));
        if (j<2) response->print(",");
      }
      response->print("],\"restPos\":");
      response->print(pointToStr(badger.legs[i]->_restPos));
      response->print(",\"_fwdPose\":");
      response->print(pointToStr(badger.legs[i]->_fwdPose.p));
      response->print(",\"_target\":");
      response->print(pointToStr(badger.legs[i]->_target));
      response->print("}");

      if (i<5) response->print(",\n");
    }

    response->print("]}");


    //send the response last
    request->send(response);
  });

  server.on("/gait", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");

    response->print("{\"legs\":[\n");

    for (uint8_t i=0; i<6; i++) {

      response->print("{");
        response->print("\"onGround\":");
        response->print(badger.legs[i]->onGround ? "true" : "false");
        response->print(",");
        response->print("\"strideF\":");
        response->print(badger.legs[i]->strideF);
        response->print(",");
        response->print("\"extension\":");
        response->print(badger.legs[i]->extension);
        response->print(",");
        response->print("\"stepUrgency\":");
        response->print(badger.legs[i]->stepUrgency);
        response->print(",");
        response->print("\"numInvalidKSG\":");
        response->print(badger.legs[i]->numInvalidKSG);
        response->print(",");
        response->print("\"numInvalidKSS\":");
        response->print(badger.legs[i]->numInvalidKSS);
      response->print("}");

      if (i<5) response->print(",\n");
    }

    response->print("]}");


    //send the response last
    request->send(response);
  });

  server.on("/setMode", HTTP_GET, [] (AsyncWebServerRequest *request) {

    AsyncWebParameter* p = request->getParam("mode");
    uint8_t m = p->value().toInt();

    newMode = m;
    newModeRequested = true;

    request->send(200, "text/plain", "OK");
  });

  server.on("/setGaitLegsRaised", HTTP_GET, [] (AsyncWebServerRequest *request) {

    AsyncWebParameter* p = request->getParam("raised");
    uint8_t v = p->value().toInt();

    newGaitLegsRaised = v;
    newGaitRequested = true;

    request->send(200, "text/plain", "OK");
  });

  server.on("/get.dat", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    for (int i=0; i < LIDAR_NUM_SEGMENTS; i++) {
      response->print((int) round(badger.lidarController.dists[i]));
      response->print("\n");
    }

    request->send(response);
  });

  server.on("/status", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    AprilTag *tag = &badger.cam.tags[badger.cam.lastSeenID];

    response->print("{");
      response->print("\"FPS\":");
      response->print(fps);
      response->print(",");
      response->print("\"ready\":");
      response->print(badger.ready ? "true" : "false");
      response->print(",");
      response->print("\"error\":");
      response->print(badger.error ? "true" : "false");
      response->print(",");
      response->print("\"errorMsg\":\"");
      response->print(badger.errorMsg);
      response->print("\",");
      response->print("\"mode\":");
      response->print(badger.robotMode);
      response->print(",");
      response->print("\"voltage\":");
      response->print(badger.voltage);
      response->print(",\n");
      response->print("\"heading\":");
      response->print(badger.lidarController.heading);
      response->print(",");
      response->print("\"pressure\":");
      response->print(pointToStr(badger.lidarController.calcPressureFromLidar(350,100)));
      response->print(",");
      response->print("\"tv\":");
      response->print(pointToStr(badger.tv));
      response->print(",");
      response->print("\"rv\":");
      response->print(badger.rv);
      response->print(",");
      response->print("\"newtv\":");
      response->print(pointToStr(badger.newtv));
      response->print(",");
      response->print("\"newrv\":");
      response->print(badger.newrv);
      response->print(",");
      response->print("\"legsRaised\":");
      response->print(badger.gaitLegsRaised);
      response->print(",");
      response->print("\"stepDuration\":");
      response->print(badger.stepDuration);
      response->print(",");
      response->print("\"maxStride\":");
      response->print(badger.maxStride);
      response->print(",");
      response->print("\"tagDetected\":");
      response->print(tag->detected ? "true" : "false");
    response->print("}");

    request->send(response);
  });


  server.on("/heading", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    response->print(badger.lidarController.heading);
    response->print("\n");

    request->send(response);
  });

  server.on("/voltage", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    response->print(badger.voltage);
    response->print("\n");

    request->send(response);
  });

  server.on("/tag", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    AprilTag *tag = &badger.cam.tags[badger.cam.lastSeenID];

    response->print("{");
      response->print("\"detected\":");
      response->print(tag->detected ? "true" : "false");
      response->print(",");
      response->print("\"tx\":");
      response->print(String(tag->tx));
      response->print(",");
      response->print("\"ty\":");
      response->print(String(tag->ty));
      response->print(",");
      response->print("\"tz\":");
      response->print(String(tag->tz));
      response->print(",");
      response->print("\"rx\":");
      response->print(String(tag->rx));
      response->print(",");
      response->print("\"ry\":");
      response->print(String(tag->ry));
      response->print(",");
      response->print("\"rz\":");
      response->print(String(tag->rz));
    response->print("}");

    request->send(response);
  });

  server.on("/torque", HTTP_GET, [] (AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");

    response->print("{\"legs\":[\n");

    for (uint8_t i=0; i<6; i++) {

      response->print("{");
        response->print("\"joints\":[\n");

        for (uint8_t j=0; j<3; j++) {
          response->print("{");
            response->print("\"maxTorque\":");
            response->print(String(badger.legs[i]->maxTorques[j]));
            response->print(",");
            response->print("\"atMaxTorque\":");
            response->print(badger.legs[i]->atMaxTorque[j] ? "true" : "false");
            response->print(",");
            response->print("\"maxTorqueTimer\":");
            response->print(String((millis() - badger.legs[i]->maxTorqueTimer[j])/1000.0f));
          response->print("}");
        }


      response->print("]}");

      if (i<5) response->print(",\n");
    }

    response->print("]}");


    //send the response last
    request->send(response);
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");



  server.begin();

  statusDisplay.bargraph(0.7);

  // init servos

  statusDisplay.bargraph(1);

  Serial.println("[] Starting updateTask");

  /*
  xTaskCreatePinnedToCore(

    updateTask,        //Function to implement the task
    "update",            //Name of the task
    5000,                   //Stack size in words
    0,                   //Task input parameter
    2,           //Priority of the task
    &_updateTask,                 //Task handle.
    0);              //Core where the task should run
  */

  esp_task_wdt_init(5,0);

  Serial.println("[] Setup done");

  startTime = millis();
}


unsigned long fpsTimer;
unsigned long debugTimer;
unsigned long loopTime;

void loop()
{
  loopTime = millis();

  OTAMgr.loop();

  // update lidar
  //badger.lidarController.update();

  if (loopTime > fpsTimer + 1000) {
    Serial.print("FPS: ");
    fps = _updateCount / ((loopTime - fpsTimer) / 1000.0);
    Serial.print(fps);

    Serial.print(", newTV: ");
    //Serial.println(pointToStr(badger.newtv));

    _updateCount = 0;
    fpsTimer = loopTime;
  }

  if (loopTime > debugTimer + 1000) {
    /*
    Serial.println("Joints: ");

    for (uint8_t leg = 0; leg<6; leg++) {
      Serial.print(leg);
      Serial.print(":  ");

      for (uint8_t i=0; i<3; i++) {
        Serial.print(badger.legs[leg]->_jointUnits[i]);
        if (i<2) Serial.print(", ");
      }
      Serial.println();
    }

    Serial.println();

    Serial.println("Joints Angles: ");

    for (uint8_t leg = 0; leg<6; leg++) {
      Serial.print(leg);
      Serial.print(":  ");

      for (uint8_t i=0; i<3; i++) {
        Serial.print((badger.legs[leg]->_jointAngles[i] * 180 / PI));
        if (i<2) Serial.print(", ");
      }
      Serial.println();
    }

    Serial.println();
    */

    debugTimer = loopTime;
  }

}
