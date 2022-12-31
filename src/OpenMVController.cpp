#include "OpenMVController.h"

OpenMVController::OpenMVController() {
  for (uint8_t i=0; i<NUM_APRILTAGS; i++) {
    tags[i].detected = false;
  }
  lastSeenID = 0;
}

void OpenMVController::init() {
  interface.enableIntTx(false);
  interface.begin(57600, SWSERIAL_8N1, 15, 4, false);
}

void OpenMVController::ageTags() {
  unsigned long m = millis();
  for (uint8_t i=0; i<NUM_APRILTAGS; i++) {
    if (m - tags[i].lastSeen > 1000)
      tags[i].detected = false;
  }
}

boolean OpenMVController::isTracking() {
  return tags[lastSeenID].detected;
}

void OpenMVController::update() {
  ageTags();

  const uint8_t pBufSize = 16;

  static uint8_t state = 0;

  static float params[7];  // id, tx, ty, tz, rx, ry, rz
  static char pbuf[pBufSize];  // buffer to hold param characters
  static uint8_t pc = 0;  // character index
  static uint8_t pn = 0;  // param number

  // check for new tag info
  int avail = interface.available();
  for (int i = 0; i < avail; ++i)
  {
      char c = (char)interface.read();

      if (c == 10) {

        // store the last decoded param
        pbuf[pc] = 0; // terminate
        //Serial.println(pbuf);
        params[pn] = atof(pbuf);

        // see if we decoded an entire tag structure
        for (uint8_t j=0; j < 7; j++) {
          //Serial.print(params[j]);
          //Serial.print(",");
        }

        if (pn == 6) {
          //Serial.print("Tag decoded: ");
          //Serial.println(params[0]);

          // update tag array
          uint8_t id = round(params[0]);
          if (id<NUM_APRILTAGS) {
            tags[id].detected = true;
            tags[id].lastSeen = millis();
            tags[id].tx = params[1] - 0.3;  // calibrated error
            tags[id].ty = params[2];
            tags[id].tz = params[3];
            tags[id].rx = params[4];
            tags[id].ry = params[5] > 180 ? -(360 - params[5]) : params[5];
            tags[id].rz = params[6];
            lastSeenID = id;
          }

        } else {
          // didn't decode a full tag ?!?
          //Serial.print("Decoded partial tag, pn: ");
          //Serial.println(pn);
        }

        // reset
        state = 0;
        pn = 0;
        pbuf[0] = 0;
        pc = 0;
      }

      switch(state) {
        case 0: // waiting for start character
          if (c == 't') state = 1;
          break;
        case 1: // decoding params, move to next when we see a comma
          if (c == ',') {
            if (pn < 7) {
              // convert to float
              pbuf[pc] = 0; // terminate
              //Serial.println(pbuf);
              params[pn] = atof(pbuf);
            } else {
              // too many params - wtf?!
              //Serial.print("Too many params: ");
              //Serial.println(pn);

            }

            pbuf[0]=0;
            pc = 0;
            pn++;
          } else {
            pbuf[pc] = c;
            pc++;
            if (pc > pBufSize-1) pc = pBufSize-1;
          }
          break;
      }
  }
}
