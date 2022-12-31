#ifndef BADGER_CONTROLLER_
#define BADGER_CONTROLLER_

#include "Arduino.h"
#include <Geometry.h>
#include "LegController.h"
#include "LidarController.h"
#include "OpenMVController.h"


#define ROBOT_MODE_PASSIVE   0
#define ROBOT_MODE_SAFE      1   // legs curled up
#define ROBOT_MODE_STAND     2   // all legs to rest position
#define ROBOT_MODE_WALK      3


struct LegGroup {
  float stepUrgency;  // max of the urgencies in the group
  boolean needToStep;
  boolean onGround;
};


class BadgerController {
public:

  LidarController lidarController;

  OpenMVController cam;

  LegController *legs[6];

  LegGroup groups[6];
  uint8_t lastGroupToStep;

  Point prevBodyOffset;  // for tweening

  unsigned long tweenStart;
  unsigned long tweenDuration;

  Point bodyOffset;
  Rotation bodyRotation;
  Rotation bodyOppRot;  // opposite of bodyRotation

  uint8_t robotMode;

  boolean ready;
  boolean error;
  String errorMsg;

  // timing
  unsigned long lastUpdate;
  unsigned long batteryTimer;

  float voltage;

  // progressive torque checks... track which id we checked last
  uint8_t lastServoId;

  // gait stuff
  Point zAxis;
  int gaitLegsRaised;  // how many legs to raise at once
  float gaitDutyCycle; // % of legs raised at once
  float gaitSteps;  // nominal numbers of steps to complete a gait cycle
  float maxStride; // nominal stride length in mm
  float stepDuration; // nominal step time in seconds

  Point tv;  // translational velocity in mm/s
  float rv;  // rotational velocity in radians/s about robot origin, + is clockwise

  Point newtv;
  float newrv;

  double strideAng;
  double angVel;

  BadgerController(DynamixelInterface &interface);

  boolean init();

  void updateGaitParams();

  void setGaitLegsRaised(int v);

  // estimte a rotational velocity in radians/s based on a target mm/s
  float estimateRotationVelocity(float v);

  void setVelocities(Point _newtv, float _newrv);
  void calcStrideParameters();

  void updatePassive();
  void updateSafe();
  void updateStand();

  // calc distance from toe of leg1 to potential collisions with leg2
  float legDistToLeg(uint8_t leg1, uint8_t leg2);

  boolean legOnGround(int legIndex);
  void calcLegStrideParams();

  boolean updatePivotUsingCamera();

  void updatePivotUsingLidar();

  boolean extensionPrioritySort(int first, int second);

  void updateWalk();

  void updateVoltage();

  void update();

  void setMode(uint8_t newMode);

private:
  DynamixelInterface &servoInterface;

  boolean _initComplete;

};

#endif
