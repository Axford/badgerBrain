#ifndef LIDAR_CONTROLLER_
#define LIDAR_CONTROLLER_

#include "Arduino.h"
#include <RPLidar.h>
#include "Geometry.h"

#define LIDAR_MOTOR_PIN   21

#define LIDAR_NUM_SEGMENTS  72
#define LIDAR_SEGMENT_WIDTH   (360 / LIDAR_NUM_SEGMENTS)

// LIDAR PWM uses the ledc module in the ESP32
#define LIDAR_PWM_FREQ         5000
#define LIDAR_PWM_CHANNEL      0
#define LIDAR_PWM_RESOLUTION   8

class LidarController {
public:
  boolean active;  // set true if scanning

  RPLidar lidar;

  int lastAng;

  float dists[LIDAR_NUM_SEGMENTS];

  TaskHandle_t _Task1;

  int robotSegWidth;
  float minObstacleDist;

  float leftSideDist; // closest obstacle on left side
  float rightSideDist;  // closest obstable on right side


  boolean validHeading;  // true if heading is passable
  float heading;  // heading in radians

  LidarController();

  void stop();
  void start();

  // angular distance between two angles
  float angDist(float a1, float a2);

  // angular distances in segments
  int segDist(int seg1, int seg2);


  // get a safe index to a segment
  int indexWithWrap(int i);

  // determine the minimum distance within arcWidth radians, centred on centraAngle
  float minDistInArc(float centreSegment, float arcWidthInSegs);

  void updateHeading();

  Point calcPressureFromLidar(float threshold, float v);

  void update();  // call frequently within a task

  static void updateTask(void *pvParameters);

};



#endif
