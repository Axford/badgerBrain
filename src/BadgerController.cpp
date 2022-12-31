#include "BadgerController.h"
#include "Arduino.h"


BadgerController::BadgerController(DynamixelInterface &interface): servoInterface(interface) {

  robotMode = ROBOT_MODE_PASSIVE;
  ready = false;
  error = false;
  _initComplete = false;
}

boolean BadgerController::init() {

  zAxis.Z() = 1;
  voltage = -1;

  Serial.println(F("[BADGER] updateGaitParams"));
  maxStride = 100;
  stepDuration = 0.0f;  // this will be calculated from velocities
  gaitLegsRaised = 3;
  lastGroupToStep = 0;
  tv(0) = 0;
  tv(1) = 100;
  rv = 0;
  lastServoId = 18;

  // init leg controllers
  Serial.println(F("[BADGER] init leg controllers"));
  for (uint8_t i=0; i<6; i++) {
    legs[i] = new LegController(i);
    if (!legs[i]->init(servoInterface)) {
      Serial.println(F("[BADGER] Error initialising leg controller"));
      error = true;
      errorMsg = "Error initialising leg "+ String(i);
    }
  }

  if (error) return false;

  updateGaitParams();

  // walk ~forward by default
  //Serial.println(F("[BADGER] set pivot"));
  //setPivot(newPivot);
  calcStrideParameters();

  Serial.println(F("[BADGER] Starting cam serial"));
  cam.init();

  _initComplete = true;
  return true;
}

void BadgerController::updateGaitParams() {
  gaitDutyCycle = gaitLegsRaised/6.0f;
  gaitSteps = round(1 / gaitDutyCycle); // how many steps to complete a cycle

  // re-assign leg groups
  uint8_t lg = 0;
  for (uint8_t i=0; i<6; i++) {
    legs[i]->legGroup = lg;
    lg++;
    if (lg >= gaitSteps) {
      lg = 0;
    }
  }
}

float BadgerController::estimateRotationVelocity(float v) {
  Point p = legs[0]->_restPos;
  // get radius
  float r = p.Magnitude();
  // simplified from (PI * v / (2 * PI * r / 2))
  return v / r;
}

void BadgerController::setGaitLegsRaised(int v) {
  if (!_initComplete) return;
  
  if (v < 1) v = 1;
  if (v > 3) v = 3;
  if (v == gaitLegsRaised) return;

  gaitLegsRaised = v;
  updateGaitParams();
}

void BadgerController::calcStrideParameters() {

  // estimate stepDuration based on requested velocities
  // use leg 0 restpos as the reference
  Point p = legs[0]->_restPos;

  // lets move it by the desired amount
  p.RotateZ(rv);
  p = p + tv;

  // now calculate distance travelled in one second
  p = p - legs[0]->_restPos;
  float d = p.Magnitude();

  // now calculate required step duration based on maxStride
  if (d > 0) {
    stepDuration = (maxStride / d) / (gaitSteps-1);
  } else {
    stepDuration = 1000; // make it a long value for the sake of it
  }
}


void BadgerController::setVelocities(Point _newtv, float _newrv) {
  tv = _newtv;
  rv = _newrv;

  calcStrideParameters();
}


void BadgerController::updatePassive() {
  // update gait
  // are all legs ready?
  uint8_t readyCount = 0;
  for (uint8_t i=0; i<6; i++) {
    if (legs[i]->ready) readyCount++;
  }

  if (!ready && readyCount == 6) {
    ready = true;
    Serial.println("READY");
  }
}


void BadgerController::updateSafe() {

}


void BadgerController::updateStand() {
  // move bodyOffset for fun
  unsigned long m = millis();

  Point offset;

  if (m > tweenStart + tweenDuration) {
      offset = bodyOffset;

      // ensure onGround is set
      for (uint8_t i=0; i<6; i++) {
          legs[i]->onGround = true;
      }

      // tilt up/down
      /*
      bodyOppRot.FromEulerAngles(
        0.1 * sin(1 * (m - tweenStart - tweenDuration) / 1000.0f),
        0,
        0);
        */
  } else {
    // tween
    float f = (m - tweenStart) / (1.0F * tweenDuration);
    offset = prevBodyOffset + ((bodyOffset - prevBodyOffset) * f);
  }

  for (uint8_t i=0; i<6; i++) {
    //legs[i]->setMode(LEG_MODE_MANUAL);

    legs[i]->_neutralTarget = legs[i] -> _restPos;

    // adjust for body offsets
    Point p;
    p = legs[i]->_neutralTarget;
    p -= offset;

    // negate body rotation
    p = bodyOppRot * p;

    legs[i]->setWorldTarget(p);
  }
}


int neighbourLegIndex(int legIndex) {
  if (legIndex < 0) legIndex += 6;
  if (legIndex >= 6) legIndex -= 6;
  return legIndex;
}

// calc distance from toe of leg1 to potential collisions with leg2
float BadgerController::legDistToLeg(uint8_t leg1, uint8_t leg2) {
  Point toe1 = legs[leg1]->_neutralTarget;
  Point coxa2 = legs[leg2]->coxaP;
  Point line2 = legs[leg2]->_neutralTarget - coxa2;
  Point coxaToToe = toe1 - coxa2;

  line2.Normalise();

  float d = coxaToToe.DotProduct(line2);

  if (d > 0) {
    Point closestPoint = line2;
    closestPoint *= d;
    closestPoint += coxa2;

    Point closestVector = toe1 - closestPoint;

    return closestVector.Magnitude();
  }

  return 100;  // closest point is "behind" coxa
}


boolean BadgerController::legOnGround(int legIndex) {
  return legs[neighbourLegIndex(legIndex)]->onGround;
}

void BadgerController::calcLegStrideParams() {
  for (uint8_t i=0; i<6; i++) {

    if (legs[i]->onGround) {

      // _nextPos should hold the toe down position of the last step
      // and thus the start of the stride
      Point v4 = legs[i]->_nextPos - legs[i]->_neutralTarget;
      float sf = 0;
      if (maxStride >0) sf = v4.Magnitude() / (maxStride);

      legs[i]->strideF = sf;


    } else {
      legs[i]->strideF = 0; // in prep for next onGround stage
    }

  }
}

Point pointOnCurve(float u, Point &p0, Point &p1, Point &p2, Point &p3 ) {
  Point p;
  for (uint8_t i=0; i<3; i++) {
    p(i) = pow(1-u,3)*p0(i) + 3*u*(1-u)*(1-u)*p1(i) + 3*(u*u)*(1-u)*p2(i) + pow(u,3)*p3(i);
  }
  return p;
}

boolean BadgerController::updatePivotUsingCamera() {

  // get current overall velocity... try to keep this constant
  float v = tv.Magnitude();

  // TODO: fix this dirty hack
  //stepDuration = 0.7f;

  if (cam.isTracking()) {
    // we've seen a tag, lets try and move towards it

    AprilTag *tag = &cam.tags[cam.lastSeenID];

    if (tag->tz >= -5) {
      // we're pretty close to the tag, lets try and rotate to square up to it

      newrv = estimateRotationVelocity(v);

      if (tag->ry < 0) newrv = -newrv;

      // vector from imagined centre of tag to robot origin
      Point tagVec;
      tagVec(1) = -350;

      tagVec.RotateZ(newrv);
      tagVec(1) += 350;

      newtv = tagVec;



      // if we're close to target, reduce the speed of movement
      if (abs(tag->ry) < 5) {
        v *=  max(1 / (5 - abs(tag->ry)), 0.2f);
      }

      // fine tune distance to tag, target is -5
      // -5 - -3 = -2
      // -5 - -7 = +2
      newtv(1) += v * ((-5) - tag->tz);

      // limit speed
      float scaling = (v / newtv.Magnitude());
      newtv *= scaling;
      newrv *= scaling;

      // ensure we're still looking at the tag
      if (tag->tx > 0) {
        // tag is to the right
        newrv = newrv - 0.1*estimateRotationVelocity(v);
      } else {
        // tag is to the left
        newrv = newrv + 0.1*estimateRotationVelocity(v);
      }

    } else if (tag->tz < -5) {
      if (abs(tag->tx) < 2) {
        // roughly centred, lets walk toward it
        newtv(1) = v;
        newtv(0) = 0;

        // also strafe a little
        if (tag->ry > 10 || tag->ry < 350) {
          if (tag->ry < 180) {
            // we're off to the left, need to strafe right
            newtv(0) = v/3;
          } else {
            // we're off to the right, need to strafe left
            newtv(0) = -v/3;
          }
        }
      } else {
        // lets mostly orient to face the tag
        newtv(0) = 0;
        newtv(1) = v/3;
        if (tag->tx > 0) {
          // tag is to the right
          newrv = - estimateRotationVelocity(v);
        } else {
          // tag is to the left
          newrv = estimateRotationVelocity(v);
        }
      }
    }

    return true;

  } else
    return false;
}


void BadgerController::updatePivotUsingLidar() {

  float v = tv.Magnitude();
  newtv = tv;
  newrv = rv;

  Point pressure = lidarController.calcPressureFromLidar(350, v);

  if (lidarController.heading == 0 && lidarController.validHeading) {

    if (!updatePivotUsingCamera()) {

    }

    newtv += pressure;

  } else {

    if (!updatePivotUsingCamera()) {

      // rotate to new heading
      newrv = estimateRotationVelocity(v);
      //if (newrv < 1) newrv = 1;
      newrv = (lidarController.heading > 0) ? -newrv : newrv;
    }

    newtv += pressure;
    if (!cam.isTracking() && newtv(1) > 0) newtv(1) = 0; // make sure we don't race forward
  }

  // ensure total magnitude is constant
  if (!cam.isTracking()) newtv = newtv * (v / newtv.Magnitude());

  if (!newtv.EqualTo(tv) || rv != newrv ) {
    tv = newtv;
    rv = newrv;

    calcStrideParameters();
  }
}


boolean BadgerController::extensionPrioritySort(int first, int second) {
  return (groups[first].stepUrgency > groups[second].stepUrgency);
}


void BadgerController::updateWalk() {

  // store current velocities, so they can be restored later
  Point oldtv = tv;
  float oldrv = rv;

  // timing
  unsigned long t = millis();
  unsigned long td = t - lastUpdate;
  if (td > 100) td = 100;  // safety
  float tdf = td / 1000.0f; // fraction of 1 second
  lastUpdate = t;

  //Serial.println(td);  // 23 - 80 ms

  // update pivot based on LIDAR heading
  if (!lidarController.validHeading) {
    setMode(ROBOT_MODE_STAND);
    return;
  }

  updatePivotUsingLidar();

  // tilt up/down
  /*
  bodyOppRot.FromEulerAngles(
    0.1 * sin(1 * (t - tweenStart - tweenDuration) / 1000.0f),
    0,
    0);
    */

  // TODO: only update this when neutralTargets change
  // and only for legs that are on the ground, as its irrelevant for legs that are stepping
  //Serial << "calcStrideParams\n";
  calcLegStrideParams();

  // reset collision checks
  for (uint8_t i=0; i<6; i++) {
    legs[i]->aboutToCollide = false;
  }

  // reset leg groups
  for (uint8_t i=0; i<gaitSteps; i++) {
    groups[i].stepUrgency = 0;
    groups[i].needToStep = false;
    groups[i].onGround = true;
  }

  // calculate motions within this time step
  Point tvs = tv * tdf;
  float rvs = rv * tdf;

  // TODO: make this dynamic
  Point zOffset;
  zOffset.Z() = 80;

  // check to see how many legs have reached their extension limit
  // and are near their heel point
  uint8_t needToStep = 0;
  uint8_t raised = 0;
  float maxStrideF = 0;
  float minStrideF = 1;
  float maxExtension = 0;
  float maxUrgency = 0;

  //Serial << "calc which legs need to step\n";
  for (uint8_t i=0; i<6; i++) {
    if (legs[i]->onGround) {
      if (legs[i]->strideF < minStrideF) minStrideF = legs[i]->strideF;
      if (legs[i]->strideF > maxStrideF) maxStrideF = legs[i]->strideF;

      maxUrgency = max(maxUrgency, legs[i]->stepUrgency);

      if (legs[i]->stepUrgency > groups[legs[i]->legGroup].stepUrgency)
        groups[legs[i]->legGroup].stepUrgency = legs[i]->stepUrgency;

      if (legs[i]->extension > maxExtension) {
        maxExtension = legs[i]->extension;
      }

      // see if we're about to collide with neighbouring legs
      if (!legs[i]->aboutToCollide) {
        float d1 = legDistToLeg(i, neighbourLegIndex(i-1));
        float d2 = legDistToLeg(i, neighbourLegIndex(i+1));

        if (d1 < 80 || d2 < 80) {
          legs[i]->aboutToCollide = true;
          if (d1 < d2) {
              legs[neighbourLegIndex(i-1)]->aboutToCollide = true;
          } else {
              legs[neighbourLegIndex(i+1)]->aboutToCollide = true;
          }
          maxUrgency += 2;
          //setMode(ROBOT_MODE_PASSIVE);
          error = true;
          errorMsg = "collision " +String(min(d1,d2)) + ": " + String(i) + " with " + (d1 < 70 ? String(neighbourLegIndex(i-1)) : String(neighbourLegIndex(i+1)));
          //return;
        }
      }

      if (legs[i]->onGround && ((legs[i]->strideF > 1) || (legs[i]->stepUrgency>1) || legs[i]->aboutToCollide)) {
        legs[i]->needToStep = true;
        groups[legs[i]->legGroup].needToStep = true;
        needToStep++;

      }
    } else {
      raised++;
      legs[i]->needToStep = false; // sanity
      groups[legs[i]->legGroup].onGround = false;

      // TODO: what about in air collisions?
      if (!legs[i]->aboutToCollide) {
        float d1 = legDistToLeg(i, neighbourLegIndex(i-1));
        float d2 = legDistToLeg(i, neighbourLegIndex(i+1));

        if (d1 < 80 || d2 < 80) {
          legs[i]->aboutToCollide = true;
          if (d1 < d2) {
              legs[neighbourLegIndex(i-1)]->aboutToCollide = true;
          } else {
              legs[neighbourLegIndex(i+1)]->aboutToCollide = true;
          }
          maxUrgency += 2;
        }
      }

      if (legs[i]->aboutToCollide) {
        // put leg straight down
        legs[i]->_nextPos = legs[i]->_neutralTarget;
        legs[i]->_nextPos.Z() = 0;
        legs[i]->_nextControlPoint = legs[i]->_nextPos + zOffset;
        legs[i]->strideF = 1;  // force immediate end to step
      }
    }
  }

  // order leg groups by stepUrgency
  uint8_t priorityOrder[6];
  for (uint8_t i=0; i<gaitSteps; i++) {
    priorityOrder[i] = i;
  }

  // insert sort
  for (uint8_t i = 1; i < gaitSteps; i++) {
    for (uint8_t j = i; j > 0 && (extensionPrioritySort(priorityOrder[j-1], priorityOrder[j]) != true); j--) {
      uint8_t tmp = priorityOrder[j-1];
      priorityOrder[j-1] = priorityOrder[j];
      priorityOrder[j] = tmp;
    }
  }

  //Serial << "maxExtension: " << maxExtension << '\n';
  //Serial << "maxStrideF: " << maxStrideF << '\n';


  //Serial << "needToStep: " << needToStep << '\n';
  //Serial << "raised: " << raised << '\n';

  // check to see if too many legs need to step
  // relative to the duty cycle... and therefore we need
  // to slow down
  boolean rapidStep = false;
  if ((needToStep + raised > gaitLegsRaised) || (maxUrgency > 1)) {
    if (maxUrgency > 0.95) {
      tvs(0) = 0;
      tvs(1) = 0;
      rvs = 0;
      rapidStep =true;
    }
  }

  //Serial << "AngToMove: " << angToMove << '\n';

  // gait planner
  uint8_t stepsPlanned = 0;



  //Serial << "plan steps\n";
  if (needToStep > 0 && (raised < gaitLegsRaised)) {

    /*
    Serial.print("Priority order: ");
    for (uint8_t i=0; i<6; i++) {
      Serial.print(priorityOrder[i]);
      Serial.print("(");
      Serial.print(legs[priorityOrder[i]]->needToStep);
      Serial.print(",");
      Serial.print(legs[priorityOrder[i]]->extension);
      Serial.print("),");
    }
    Serial.println();
    */

    // for each leg Group, in priority order
    // determine which legs (if any) to step
    for (uint8_t i=0; i<gaitSteps; i++) {
      uint8_t groupIndex = priorityOrder[i];

      // && (lastGroupToStep != groupIndex)
      if (groups[groupIndex].needToStep && groups[groupIndex].onGround ) {

        // check other groups are on the ground
        boolean goodToStep = true;
        for (uint8_t j=0; j<gaitSteps; j++) {
          if ((i!=j) && (!groups[j].onGround)) goodToStep = false;
        }

        if (goodToStep) {
          lastGroupToStep = groupIndex;

          groups[groupIndex].onGround = false;

          // for each leg
          for (uint8_t j=0; j<6; j++) {
            uint8_t legIndex = j;

            if (legs[legIndex]->legGroup == groupIndex) {
              stepsPlanned++;

              legs[legIndex]->_neutralTarget.Z() = 0; // to be sure
              legs[legIndex]->_lastPos = legs[legIndex]->_neutralTarget;

              // take a step - move toward toe
              // but only move to a gait cycle past the minStrideF point
              float strideF = minStrideF - (1/(gaitSteps-1));
              //float strideF = 0;
              if (strideF < 0) strideF = 0;
              if (strideF > 0.5) strideF = 0;

              // update restPos if bodyHeight has changed
              // TODO: only update this if needed
              legs[legIndex]->updateRestPos( bodyOffset.Z() );


              Point p = (legs[legIndex]->_restPos);
              p.RotateZ(rv);
              p = p + tv;

              // now scale such that restpos -> p is stride/2 or less
              Point restToNext = p - legs[legIndex]->_restPos;

              // find a valid/reachable target, working back from maxStride/2
              Point candidate;
              Point candidateWorld;
              // strideF will be in range 0..0.5
              float candidateDist = (0.5 - strideF) * maxStride/2;
              KinematicSolution ks;
              uint8_t iter = 0;

              do {
                candidate = restToNext * (candidateDist / restToNext.Magnitude());
                candidate += legs[legIndex]->_restPos;

                // adjust for body offsets
                candidateWorld = candidate - bodyOffset;

                // negate body rotation
                candidateWorld = bodyOppRot * candidateWorld;

                ks = legs[legIndex]->reverseKinematics( legs[legIndex]->worldToLocal(candidateWorld) );
                candidateDist *= 0.8; // reduce in 20% increments
                iter++;
              } while (!ks.valid && iter < 10);

              if (iter == 10) {
                error = true;
                errorMsg = "No KS " + String(legIndex) + ": " + String(candidateDist);
              }

              legs[legIndex]->_nextPos = candidate;

              // calc bezier control points for step curve
              legs[legIndex]->_lastControlPoint = legs[legIndex]->_lastPos + zOffset;
              legs[legIndex]->_nextControlPoint = legs[legIndex]->_nextPos + zOffset;


              legs[legIndex]->setMode(LEG_MODE_STEP);
              legs[legIndex]->needToStep = false;
              legs[legIndex]->lastStepT = t;
              legs[legIndex]->stepF = 0;
            }
          }

          // no other groups can step if this one is stepping
          // so break
          break;
        }

      }
    }
  }


  //Serial << "Leg motion planning\n";

  // now update motion planning for each leg
  for (uint8_t i=0; i<6; i++) {
    if (legs[i]->onGround) {
      Point p = legs[i]->_neutralTarget;
      p.RotateZ(-rvs);
      legs[i]->_neutralTarget = p - tvs;


    } else {
      //legs[i]->stepF = (t - legs[i]->lastStepT) / (1000.0f * stepDuration);

      // to account for a stepDuration that is constantly changing
      legs[i]->stepF += tdf / (rapidStep ? 0.5f : stepDuration);

      if (legs[i]->stepF < 0) legs[i]->stepF = 0;
      if (legs[i]->stepF > 1) legs[i]->stepF = 1;

      // get point along bez curve from lastPos to nextPos via handles
      legs[i]->_neutralTarget  = pointOnCurve(legs[i]->stepF, legs[i]->_lastPos, legs[i]->_lastControlPoint, legs[i]->_nextControlPoint, legs[i]->_nextPos);

      // temporary lerp
      //legs[i]->_neutralTarget = legs[i]->_lastPos + ((legs[i]->_nextPos - legs[i]->_lastPos) * stepF);

      if (legs[i]->stepF >= 1) {
        // step complete
        legs[i]->setMode(LEG_MODE_STAND);
      }


    }

    // adjust for body offsets
    Point p1;
    p1 = legs[i]->_neutralTarget;
    p1 -= bodyOffset;

    // negate body rotation
    p1 = bodyOppRot * p1;

    legs[i]->setWorldTarget(p1);

  }

  //Serial << "updateWalk complete\n\n";

  tv = oldtv;
  rv = oldrv;
}


void BadgerController::update() {

  if (!_initComplete) return;
  // lets not update too fast!
  //static long lastUpdate = 0;

  //if (millis() < lastUpdate + 20) return; // limit update rate
  //lastUpdate = millis();

  // check in on camera
  cam.update();

  // get updated torques - check one servo each time update is called
  lastServoId++;
  if (lastServoId > 18) lastServoId = 1;
  for (uint8_t i=0; i<6; i++) {
    legs[i]->getTorqueForId(lastServoId);
  }

  updateVoltage();

  switch(robotMode) {
    case ROBOT_MODE_PASSIVE:  updatePassive(); break;
    case ROBOT_MODE_SAFE: updateSafe(); break;
    case ROBOT_MODE_STAND:  updateStand(); break;
    case ROBOT_MODE_WALK: updateWalk(); break;
  }

  // update legs
  for (uint8_t i=0; i<6; i++) {
    legs[i]->update();
  }

  // now do a sync write for all servo joints
  if (robotMode != ROBOT_MODE_PASSIVE) {
    uint8_t ids[18];
    uint8_t syncData[18*2];
    uint8_t tid = 0;
    for (uint8_t i=0; i<6; i++) {
      for (uint8_t j=0; j<3; j++) {
        ids[tid] = legs[i]->config.servoIDs[j];
        syncData[tid*2] = legs[i]->_jointUnits[j] & 0xFF;  // low byte
        syncData[tid*2+1] = (legs[i]->_jointUnits[j] >> 8) & 0xFF;  // high byte
        tid++;
      }
    }

    DynamixelStatus status = servoInterface.syncWrite(
      18,
      ids,
      DYN_ADDRESS_GOAL_POSITION,
      2,
      (uint8_t*)syncData
    );

    if (status != DYN_STATUS_OK) {
      Serial.print("ge:");
      Serial.println(status, BIN);
    }
  }


}


void BadgerController::updateVoltage() {
  unsigned long m = millis();

  if (!_initComplete) return;

  if (m > batteryTimer + 1000) {
    voltage = legs[0]->getVoltage();

    if (voltage > -1 && voltage < 11.1) {
      error = true;
      errorMsg = "Low voltage";
      setMode(ROBOT_MODE_PASSIVE);
      ready = false;
    }

    batteryTimer = m;
  }
}


void BadgerController::setMode(uint8_t newMode) {

  if (!_initComplete) return;

  if (newMode == robotMode || !ready) return;

  robotMode = newMode;

  switch(robotMode) {
    case ROBOT_MODE_PASSIVE:
      for (uint8_t i=0; i<6; i++) {
        legs[i]->setMode(LEG_MODE_PASSIVE);
      }
      bodyOffset.Z() = 0; // reset
      lidarController.stop();
      break;

    case ROBOT_MODE_SAFE:
      for (uint8_t i=0; i<6; i++) {
        legs[i]->setMode(LEG_MODE_MANUAL);
        legs[i]->setRateLimit(1);
        legs[i]->updateRestPos(0);
        legs[i]->setWorldTarget(legs[i]->_restPos);
      }
      bodyOffset.Z() = 0; // reset
      lidarController.start();
      break;

    case ROBOT_MODE_STAND:
      for (uint8_t i=0; i<6; i++) {
        legs[i]->setMode(LEG_MODE_MANUAL);
        legs[i]->setRateLimit(1);
      }

      prevBodyOffset = bodyOffset;
      bodyOffset.Z() = 130;

      tweenStart = millis();
      tweenDuration = 2000;

      lidarController.start();
      break;

    case ROBOT_MODE_WALK:
      for (uint8_t i=0; i<6; i++) {
        legs[i]->setMode(LEG_MODE_STAND);
        legs[i]->setRateLimit(100);  // remove limit
      }

      //Point rtv;
      //rtv(1) = 50;
      //setVelocities(rtv, 0);

      bodyOffset.Z() = 130;
      lidarController.start();
      lastUpdate = millis();
      break;
  }
}
