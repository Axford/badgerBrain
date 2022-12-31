#include "LegController.h"
#include "Arduino.h"
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include "JsonUtils.h"


// convert degress to radians
float degToRad(float deg) {
  return deg * PI / 180;
}

float radToDeg(float rad) {
  return rad * 180 / PI;
}

float sqr(float x) {
  return x*x;
}


LegController::LegController(uint8_t id) {
  config.id = id;
  _mode = 255;  // dummy mode to trigger change to passive on init
  ready = false;
  _rateLimit = 0.1; //  slow rate limit for init
  strideF = 0;
  extension = 0;
  stepUrgency = 0;
  aboutToCollide = false;
  onGround = false; // unknown
  needToStep = false;
  numInvalidKSG = 0;
  numInvalidKSS = 0;
  restRadius = 130;

  // init vectors
  for (uint8_t i=0; i<3; i++) {
    _targetJointAngles[i] = 0;
    _jointAngles[i] = 0;
    _jointUnits[i] = 0;
    torques[i] = 0;
    maxTorques[i] = 0;
    atMaxTorque[i] = false;
  }
}

float LegController::getVoltage() {
  return _servos[0]->voltage();
}

void LegController::getTorque(uint8_t index) {
  torques[index] = _servos[index]->torque();
  float aTorque = abs(torques[index]);
  maxTorques[index] = max(maxTorques[index], aTorque);
  if (aTorque >= 0.99) {
    if (!atMaxTorque[index]) {
      maxTorqueTimer[index] = millis();
      atMaxTorque[index] = true;
    }
  } else {
    atMaxTorque[index] = false;
  }
}

void LegController::getTorques() {
  for (uint8_t i=0; i<3; i++) {
    getTorque(i);
  }
}

void LegController::getTorqueForId(uint8_t id) {
  for (uint8_t i=0; i<3; i++) {
    if (config.servoIDs[i] == id) {
      getTorque(i);
    }
  }
}

void LegController::setMode(uint8_t newMode) {
  if (newMode == _mode) return;

  _mode = newMode;
  _lastUpdate = millis();

  switch(_mode) {
    case LEG_MODE_PASSIVE:
      disable();
      break;
    case LEG_MODE_STAND:
      onGround = true;
      enable();
      break;
    case LEG_MODE_STEP:
      onGround = false;
      stepF = 0;
      enable();
      break;
    default:
      enable();
  }
}

void LegController::setRateLimit(float newLimit) {
  _rateLimit = newLimit;
}

void LegController::disable() {
  uint8_t retries = 0;
  boolean retry;

  do {
    retry = false;
    for (uint8_t i=0; i<3; i++) {
      if (_servos[i]->enableTorque(false) != DYN_STATUS_OK) retry = true;
    }
  } while (retry && retries < 5);
}

void LegController::enable() {
  for (uint8_t i=0; i<3; i++) {
    _servos[i]->enableTorque(true);
  }
}

boolean LegController::init(DynamixelInterface &interface) {
  Serial.print(F("[LEG] Init "));
  Serial.println(config.id);

  if (!SPIFFS.begin()) {
    Serial.println(F("[LEG] Error starting SPIFFS"));
    return false;
  }

  // set defaults on load in case of missing parameters
  loadConfiguration(true);
  // immediately save in case of new configuration items
  //saveConfiguration();

  coxaP.X() = config.coxaOffsets[0];
  coxaP.Y() = config.coxaOffsets[1];
  coxaP.Z() = config.coxaOffsets[2];

  forwardKinematics();

  // init restPos
  updateRestPos(0);

  _neutralTarget = _restPos;
  _lastPos = _restPos;
  _nextPos = _restPos;

  //_posedRestPos = _pose * _restPos;

  // create servo objects
  Serial.println(F("[LEG] Setup servos"));
  for (uint8_t i=0; i<3; i++) {
    Serial.print(F("ID:"));
    Serial.println(config.servoIDs[i]);
    _servos[i] = new DynamixelMotor(interface, config.servoIDs[i]);
  }

  setMode(LEG_MODE_PASSIVE);

  Serial.println(F("[LEG] Init Complete"));

  return true;
}

void LegController::updateRestPos(float bodyHeight) {

  float rmax = sqrt(
    sqr(config.boneLengths[LEG_FEMUR] + config.boneLengths[LEG_TIBIA]) -
    sqr(bodyHeight)
  );

  float rmin = config.boneLengths[LEG_FEMUR];

  restRadius = (rmax + rmin)/2 + config.boneLengths[LEG_COXA];

  _restPos.X() = config.coxaOffsets[0] + restRadius * cos(config.coxaAngle);
  _restPos.Y() = config.coxaOffsets[1] + restRadius * sin(config.coxaAngle);
  _restPos.Z() = config.coxaOffsets[2];

  KinematicSolution ks = reverseKinematics(_restPos);
  restExtension = ks.ef;
}

Transformation LegController::forwardKinematics() {
  _fwdPose = Identity<4,4>();

  // tibia
  _fwdPose.Translate(config.boneLengths[LEG_TIBIA],0,0);
  _fwdPose.RotateY(-_jointAngles[LEG_TIBIA]);

  // femur
  _fwdPose.Translate(config.boneLengths[LEG_FEMUR],0,0);
  _fwdPose.RotateY(-_jointAngles[LEG_FEMUR]);

  // coxa
  _fwdPose.Translate(config.boneLengths[LEG_COXA],0,0);
  _fwdPose.RotateZ(_jointAngles[LEG_COXA] + config.coxaAngle);

  // hip
  //_fwdPose.RotateZ(config.coxaAngle);
  _fwdPose.Translate(config.coxaOffsets[0],config.coxaOffsets[1],config.coxaOffsets[2]);

  // TODO: body pose

  return _fwdPose;
}


void LegController::loadConfigurationFromJSON(JsonObject & obj, boolean setDefaults) {
  // Copy values from the JsonDocument to the Config
  float defV[3] = {0,0,0};
  uint8_t defIntV[3] = {0,0,0};
  int defI[3] = {0,0,0};
  boolean defB[3] = {false, false, false};

  config.id = getJsonInt(obj, F("id"), (setDefaults ? 0 : config.id) );
  config.coxaAngle = degToRad( getJsonFloat(obj, F("coxaAngle"), (setDefaults ? 0 : config.coxaAngle) ) );

  getJsonIntVector(config.centres, 3, obj, F("centres"), defI);
  getJsonIntVector(config.deg45, 3, obj, F("deg45"), defI);
  getJsonBooleanVector(config.invert, 3, obj, F("invert"), defB);

  getJsonFloatVector(config.coxaOffsets, 3, obj, F("coxaOffsets"), defV);
  getJsonFloatVector(config.boneLengths, 3, obj, F("boneLengths"), defV);
  getJsonFloatVector(config.legMinAngles, 3, obj, F("legMinAngles"), defV);
  getJsonFloatVector(config.legMaxAngles, 3, obj, F("legMaxAngles"), defV);
  getJsonUInt8Vector(config.servoIDs, 3, obj, F("servoIDs"), defIntV);

  // convert angles to radians, and calc unitsPerRadian
  for (uint8_t i=0; i<3; i++) {
    config.legMinAngles[i] = degToRad(config.legMinAngles[i]);
    config.legMaxAngles[i] = degToRad(config.legMaxAngles[i]);

    config.unitsPerRadian[i] = abs(config.deg45[i] - config.centres[i]) / (PI/4.0f);
  }


  //printArray(F("coxaOffsets"), config.coxaOffsets, 3);

  Serial.println(F("[LEG] Loaded configuration:"));
  serializeJson(obj, Serial);
  Serial.println();
}

String LegController::getConfigFilename() {
  return "/leg" + String(config.id) + ".json";
}


boolean LegController::loadConfiguration(boolean setDefaults) {
  boolean loaded = false;

  String fn = getConfigFilename();
  if (SPIFFS.exists(fn)) {
    File file = SPIFFS.open(fn, FILE_READ);

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/v6/assistant to compute the capacity.
    StaticJsonDocument<768> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println(F("[LEG] Failed to read file, using default configuration"));
    } else {

      JsonObject obj = doc.as<JsonObject>();

      loadConfigurationFromJSON(obj, setDefaults);

      loaded = true;
    }

    // Close the file (Curiously, File's destructor doesn't close the file)
    file.close();
  } else {
    Serial.println(F("[LEG] Config file does not exist, creating template"));
    saveConfiguration();
  }

  return loaded;
}


void LegController::serializeConfig(JsonDocument & doc) {
  // Set the values in the document
  float tempVec[3];

  doc[F("id")] = config.id;
  doc[F("coxaAngle")] = radToDeg(config.coxaAngle);

  serializeIntVector(config.centres, 3, doc, F("centres"));
  serializeIntVector(config.deg45, 3, doc, F("deg45"));
  serializeBooleanVector(config.invert, 3, doc, F("invert"));
  serializeFloatVector(config.unitsPerRadian, 3, doc, F("unitsPerRadian"));

  serializeFloatVector(config.coxaOffsets, 3, doc, F("coxaOffsets"));
  serializeFloatVector(config.boneLengths, 3, doc, F("boneLengths"));

  for (uint8_t i=0; i<3; i++) tempVec[i] = radToDeg(config.legMinAngles[i]);
  serializeFloatVector(tempVec, 3, doc, F("legMinAngles"));

  for (uint8_t i=0; i<3; i++) tempVec[i] = radToDeg(config.legMaxAngles[i]);
  serializeFloatVector(tempVec, 3, doc, F("legMaxAngles"));

  serializeUInt8Vector(config.servoIDs, 3, doc, F("servoIDs"));
}


void LegController::saveConfiguration() {
  // Open file for writing
  File file = SPIFFS.open(getConfigFilename(), FILE_WRITE);
  if (!file) {
    Serial.println(F("[LEG] Failed to create config file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<600> doc;

  serializeConfig(doc);

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("[LEG] Failed to write config to file"));
  }

  // Close the file
  file.close();
}


void LegController::updatePassive() {
  uint8_t okCount = 0;

  // get servo angles
  for (uint8_t i=0; i<3; i++) {
    uint16_t p;

    if (_servos[i]->getCurrentPosition(p) == DYN_STATUS_OK && p < 1024) {
      _jointUnits[i] = p;

      // convert to "corrected" joint angles in radians
      _jointAngles[i] = (_jointUnits[i] - config.centres[i]) * (config.invert[i] ? -1 : 1) / config.unitsPerRadian[i];

      okCount++;
    }
  }

  forwardKinematics();

  if (!ready && okCount==3) ready = true;
}


void LegController::updateManualJoints() {
  unsigned long t = millis();

  float td = (t - _lastUpdate) / 1000.0f;
  _lastUpdate = t;

  float rls = _rateLimit * td;

  // move toward target angles, rate limit, enforce limits
  for (uint8_t i=0; i<3; i++) {
    float err = _targetJointAngles[i] - _jointAngles[i];
    if (err > rls) { err = rls; }
    else if (err < -rls) { err = -rls; }

    // update current
    _jointAngles[i] += err;

    // constrain to joint limits
    _jointAngles[i] = min( max(_jointAngles[i], config.legMinAngles[i]), config.legMaxAngles[i]);

    // convert from radians to servo units, account for inversion
    _jointUnits[i] = ( _jointAngles[i] * config.unitsPerRadian[i] ) * (config.invert[i] ? -1 : 1) + config.centres[i];

    // update servo with new goal position
    //managed by a sync write
/*
    DynamixelStatus status = _servos[i]->goalPosition(_jointUnits[i]);
    if (status != DYN_STATUS_OK) {
      Serial.print("ge:");
      Serial.println(status, BIN);
    }
  */
  }

  //forwardKinematics();
}

Point LegController::worldToLocal(Point p) {
  p = p - coxaP;

  p.RotateZ(-config.coxaAngle);

  return p;
}

void LegController::setWorldTarget(Point p) {
  setTarget( worldToLocal(p) );
}


void LegController::setTarget(Point p) {
  _target = p;
}


KinematicSolution LegController::reverseKinematics(Point t) {
  KinematicSolution tmp;
  tmp.valid = true;

  tmp.gamma = atan2(t.Y(), t.X());
  if (isnan(tmp.gamma)) {
    tmp.gamma = 0; // unable to solve
    tmp.valid = false;
  }

  // convert to a 2D problem
  float x1 = sqrt(sqr(t.X()) + sqr(t.Y())) - config.boneLengths[LEG_COXA];
  float y1 = t.Z();

  // leg length shortcuts
  float l1 = config.boneLengths[LEG_FEMUR];
  float l2 = config.boneLengths[LEG_TIBIA];

  // distance from coxa joint to toe
  float l = sqrt(sqr(x1) + sqr(y1));

  float alpha1 = acos(y1 / l);

  float alpha2 = acos( (sqr(l2) - sqr(l1) - sqr(l)) / (-2 * l1 * l));

  tmp.alpha = (alpha1 + alpha2) - PI/2;

  tmp.beta = - (PI - acos( (sqr(l) - sqr(l2) - sqr(l1)) / (-2*l1*l2) ));

  // alternate solution, invert alpha and beta
  // aim for knee up solutions, i.e. alpha > 0
  if (tmp.alpha > 0) {
      tmp.alpha = tmp.alpha - 2*alpha2;
      tmp.beta = -tmp.beta;
  }

  if (isnan(tmp.alpha) || isnan(tmp.beta)) {
    tmp.valid =false;
  }

  if (tmp.valid) {
    // check vs angle limits
    if (
      (tmp.gamma > config.legMaxAngles[LEG_COXA]) ||
      (tmp.gamma < config.legMinAngles[LEG_COXA]) ||
      (tmp.alpha > config.legMaxAngles[LEG_FEMUR]) ||
      (tmp.alpha < config.legMinAngles[LEG_FEMUR]) ||
      (tmp.beta > config.legMaxAngles[LEG_TIBIA]) ||
      (tmp.beta < config.legMinAngles[LEG_TIBIA])
    ) tmp.valid = false;
  }

  tmp.ef = l / (l1 + l2);

  return tmp;
}


void LegController::updateManual() {

  // solve inverse kinematics to reach _target (in joint coord frame)
  KinematicSolution ks = reverseKinematics(_target);

  if (ks.valid) {
    _targetJointAngles[LEG_COXA] = ks.gamma;
    _targetJointAngles[LEG_FEMUR] = ks.alpha;
    _targetJointAngles[LEG_TIBIA] = ks.beta;
  } else {
    if (onGround) {
      numInvalidKSG++;
    } else
      numInvalidKSS++;
  }

  extension = ks.ef;

  stepUrgency = extension; // 0 .. 1

  // check if invalid
  if (!ks.valid) {
    // really need to adjust step if target is unreachable
    stepUrgency += 1;
  }

  // update joints based on new target angles
  updateManualJoints();
}


void LegController::update() {

  switch(_mode) {
    case LEG_MODE_PASSIVE: updatePassive();  break;
    case LEG_MODE_MANUAL_JOINTS: updateManualJoints(); break;
    case LEG_MODE_MANUAL: updateManual(); break;
    case LEG_MODE_STAND:
    case LEG_MODE_STEP: updateManual(); break;
  }
}
