#include "LidarController.h"

LidarController::LidarController() {

  robotSegWidth = 12;
  minObstacleDist = 450;

  leftSideDist = 0;
  rightSideDist = 0;

  // use hardware serial1 for comms, as Serial2 is used for dynamixel servos
  Serial1.begin(115200, SERIAL_8N1, 19, 18);

  lidar.begin(Serial1);

  // configure ledc module
  ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RESOLUTION);
  ledcAttachPin(LIDAR_MOTOR_PIN, LIDAR_PWM_CHANNEL);

  active = false;

/*
  xTaskCreatePinnedToCore(
    this->updateTask,        //Function to implement the task
    "lidarUpdate",            //Name of the task
    5000,                   //Stack size in words
    this,                   //Task input parameter
    5,           //Priority of the task
    &_Task1,                 //Task handle.
    1);              // run on core 1, as core 0 is running the servo control stuff
    */
}


void LidarController::stop() {
  if (active) {
    ledcWrite(LIDAR_PWM_CHANNEL, 0); //stop the rplidar motor
    lidar.stop();

    active = false;
  }
}


void LidarController::start() {
  if (!active) {
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      Serial.println(F("[LIDAR] Detected, starting scan\n"));
      ledcWrite(LIDAR_PWM_CHANNEL, 255); //start the rplidar motor
      lidar.startScan();
      active = true;
    } else {
      active = false;
      Serial.println(F("[LIDAR] Error getting lidar device info\n"));
    }
  }
}

float LidarController::angDist(float a1, float a2) {
  float ad = abs(a1-a2);
  if (ad > PI) ad = PI*2 - ad;
  return ad;
}

int LidarController::segDist(int seg1, int seg2) {
  int ad = abs(seg1-seg2);
  if (ad > LIDAR_NUM_SEGMENTS/2) ad = (LIDAR_NUM_SEGMENTS-1) - ad;
  return ad;
}

int LidarController::indexWithWrap(int i) {
  if (i >= LIDAR_NUM_SEGMENTS) i-= LIDAR_NUM_SEGMENTS;
  if (i < 0) i += LIDAR_NUM_SEGMENTS;
  return i;
}

float LidarController::minDistInArc(float centreSegment, float arcWidthInSegs) {
  int startSeg = centreSegment - arcWidthInSegs/2;
  int endSeg = centreSegment + arcWidthInSegs/2;

  float minDist = dists[ indexWithWrap(startSeg) ];
  for(int i=startSeg+1; i<=endSeg; i++) {
    uint8_t index = indexWithWrap(i);
    if (dists[index] < minDist) minDist = dists[index];
  }

  return minDist;
}


void LidarController::updateHeading() {
  int minAngDistFromCentre = 100;
  float minAng = 0;

  for (uint8_t i=0; i<LIDAR_NUM_SEGMENTS; i++) {

    float centreAng = radians(i * 360/LIDAR_NUM_SEGMENTS);

    int angDistFromCentre = segDist(i, 0);

    float md = minDistInArc(i, robotSegWidth);

    if (md > minObstacleDist && angDistFromCentre < minAngDistFromCentre) {
      minAngDistFromCentre = angDistFromCentre;
      minAng = centreAng;
    }
  }

  if (minAngDistFromCentre < 100) {
    validHeading = true;

    if (minAng > PI) minAng = - (2* PI - minAng);

    heading = minAng;
  } else {
    validHeading = false;
  }

  // now calculate left side min dist
  float md = 10000;
  for (int i=LIDAR_NUM_SEGMENTS-1-robotSegWidth/2; i > LIDAR_NUM_SEGMENTS*5/8; i--) {
    if (dists[i] < md) md = dists[i];
  }
  leftSideDist = md;

  md = 10000;
  for (int i=robotSegWidth/2; i < LIDAR_NUM_SEGMENTS*3/8; i++) {
    if (dists[i] < md) md = dists[i];
  }
  rightSideDist = md;

}

// calculate a vector away from all nearby obstacles
Point LidarController::calcPressureFromLidar(float threshold, float v) {
  Point p;
  float minDist = 1000;
  float minDistAng = 0;

  for (uint8_t i=0; i<LIDAR_NUM_SEGMENTS; i++) {

    // ignore things too close... as could be invalid distances or our own legs
    if (dists[i] > 250 && dists[i] < threshold) {
      if (dists[i] < minDist) {
        minDist = dists[i];
        // store 180 deg opposite the minDist
        // and rotate everything 90 deg to account for orientation of lidar vs robot coord frame
        minDistAng = radians(i * 360/LIDAR_NUM_SEGMENTS) + PI - PI/2;
      }
    }
  }

  if (minDist < threshold) {
    p(0) = v * cos(minDistAng);
    p(1) = v * sin(minDistAng);
  }

  return p;
}


void LidarController::update() {
  if (!active) return;

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    uint8_t  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    //perform data processing here...
    if (quality >= 1) {
      float q = min(quality / 64.0, 1.0) / LIDAR_SEGMENT_WIDTH;  // 6 bit quality info

      int ang = min(floor(angle / LIDAR_SEGMENT_WIDTH), (float)(LIDAR_NUM_SEGMENTS-1));
      dists[ang] = (dists[ang] * (1-q) + distance * q);

      lastAng = ang;
    }

    if (startBit) {
      updateHeading();
    }
  }
}

void LidarController::updateTask(void *pvParameters) {
  LidarController *l_pThis = (LidarController *) pvParameters;

  for(;;){
    if (l_pThis->active) {
      l_pThis->update();
    }


    vTaskDelay(1);
  }
}
