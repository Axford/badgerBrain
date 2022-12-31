#ifndef LEG_CONTROLLER_
#define LEG_CONTROLLER_

#include "Arduino.h"
#include <Geometry.h>
#include <ArduinoJson.h>
#include <DynamixelMotor.h>

#define LEG_COXA  0
#define LEG_FEMUR 1
#define LEG_TIBIA 2

// Stand - maintain ground contact and move relative to body to maintain posture
#define LEG_MODE_STAND  0

// Step - take a step, accounting for posture/motion.  Attempt to avoid ground contact until target reached.  Transition to stand once target reached.
// follow a bezier path defined by the gait planner
#define LEG_MODE_STEP   1

//Probe - Find a way over large obstacles... what does really mean???
#define LEG_MODE_PROBE  2

// Manual - Directly control target position
#define LEG_MODE_MANUAL 3

// Manual Joints - Directly control joint targets
#define LEG_MODE_MANUAL_JOINTS 4

// Passive - servos disabled, joint can be manually moved.  Legs are initialised into passive mode
// joint positions are continuously tracked in passive mode
#define LEG_MODE_PASSIVE       5


struct LegConfig {
  uint8_t id;
  int centres[3];
  int deg45[3];
  float unitsPerRadian[3];
  boolean invert[3];
  float coxaOffsets[3];
  float coxaAngle;
  float boneLengths[3];
  float legMinAngles[3];
  float legMaxAngles[3];
  uint8_t servoIDs[3];
};


struct KinematicSolution {
  float alpha;
  float beta;
  float gamma;
  float ef;  // extension factor
  boolean valid;
};


class LegController {
public:
  LegConfig config;
  Point coxaP;  // config.coxaOffsets as a point object

  DynamixelMotor *_servos[3];

  uint8_t _mode;
  uint16_t _jointUnits[3];  // current raw joint angles read from servos
  float _jointAngles[3];  // calculated from raw angles read from servos in radians
  float torques[3];
  float maxTorques[3];
  boolean atMaxTorque[3];  // true if at max at last check
  unsigned long maxTorqueTimer[3];  // millis() when maxTorque began

  float _targetJointAngles[3];

  uint8_t legGroup; // id of leg group

  unsigned long numInvalidKSG; // how many attempts to reach an invalid target
  unsigned long numInvalidKSS;

  float extension;
  float restExtension;
  float stepUrgency;  // 0 = no urgency, big values are high urgency
  boolean aboutToCollide;
  boolean onGround;
  boolean needToStep;

  Transformation _pose;  // desired body offset and rotation
  Transformation _fwdPose;

  float restRadius;
  Point _restPos;  // in robot origin frame, prior to _pose

  Point _target;
  Point _neutralTarget;
  Point _lastPos;
  Point _nextPos;
  Point _lastControlPoint;
  Point _nextControlPoint;

  float stepF;
  unsigned long lastStepT;

  boolean strideExtending;  // true if strideF is increasing, i.e. we're past the rest point
  float strideF;

  float _rateLimit;  // radians per update TODO: change to something sensible!

  boolean ready;  // set to true once init complete and all servos have reported positions

  LegController(uint8_t id);

  void disable();
  void enable();

  void setMode(uint8_t newMode);
  void setRateLimit(float newLimit);

  // will also init sensors, ensure Wire.begin has been called first
  boolean init(DynamixelInterface &interface);

  String getConfigFilename();

  void loadConfigurationFromJSON(JsonObject & doc, boolean setDefaults);
  boolean loadConfiguration(boolean setDefaults);
  void saveConfiguration();
  void serializeConfig(JsonDocument & doc);

  void updateRestPos(float bodyHeight);

  float getVoltage();
  void getTorque(uint8_t index);
  void getTorques();
  void getTorqueForId(uint8_t id);  // get torque for servo with specified id

  Point worldToLocal(Point p);  // convert a world target to leg coordinate frame

  void setWorldTarget(Point p);  // vector relative to robot origin
  void setTarget(Point p);  // vector from coxa joint to target

  // solve kinematics for target t
  KinematicSolution reverseKinematics(Point t);

  Transformation forwardKinematics();

  void updatePassive();
  void updateManualJoints();
  void updateManual();


  void update();


private:
  unsigned long _lastUpdate;


};


#endif
