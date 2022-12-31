#ifndef STATUS_DISPLAY_
#define STATUS_DISPLAY_

#include "Arduino.h"
#include <NeoPixelBus.h>

class StatusDisplay {
public:

  StatusDisplay(uint8_t pin, uint8_t pixelCount);

  void init();

  void setColor(float h, float s);
  void setBrightness(float b);

  void bargraph(float v);

  void setPixel(uint8_t pix, RgbColor clr);

  void show();

private:
  uint8_t _pixelCount;
  RgbColor _clr;
  RgbColor _black;
  float _brightness;
  float _hue;
  float _sat;
  NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> * _strip;
};


#endif
