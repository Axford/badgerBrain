#ifndef OPEN_MV_CONTROLLER_
#define OPEN_MV_CONTROLLER_

#include "SoftwareSerial.h"
#include <Arduino.h>

#define NUM_APRILTAGS    30

class AprilTag {
public:
  boolean detected;   // true if we've recently seen this tag
  unsigned long lastSeen;  // when did we last see this tag
  float tx;
  float ty;
  float tz;
  float rx;
  float ry;
  float rz;
};


class OpenMVController {
public:
  SoftwareSerial interface;

  AprilTag tags[NUM_APRILTAGS]; // TAG16H5 set contains 30 tags, 0..29

  uint8_t lastSeenID;  // id of the last tag we've seen, for easy access to tag array

  OpenMVController();

  void init();

  void ageTags();  // set old tags to detected=false

  boolean isTracking();

  void update();
};




#endif
