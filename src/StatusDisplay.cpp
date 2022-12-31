#include "StatusDisplay.h"
#include "Arduino.h"
#include <NeoPixelBus.h>

/*

#define colorSaturation 128


RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

*/


StatusDisplay::StatusDisplay(uint8_t pin, uint8_t pixelCount) {
  _pixelCount = pixelCount;
  _brightness = 0.05f;
  _hue = 0;
  _sat = 1.0f;
  _clr = HslColor(_hue, _sat, _brightness);  // red
  _black = RgbColor(0);
  _strip = new NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>(pixelCount, pin);
}


void StatusDisplay::init() {
  Serial.println(F("[STATUS] Init display"));
  _strip->Begin();
  _strip->Show();
}

void StatusDisplay::setColor(float h, float s) {
  _hue = h / 360;
  _sat = s;
  _clr = HslColor(_hue, _sat, _brightness);
}

void StatusDisplay::setBrightness(float b) {
  _brightness = b;
  _clr = HslColor(_hue, _sat, _brightness);
}

void StatusDisplay::bargraph(float v) {
  uint8_t p = round((_pixelCount-1) * v);
  for (uint8_t i=0; i<_pixelCount; i++) {
    if (i <= p) {
      _strip->SetPixelColor(i,_clr);
    } else {
      _strip->SetPixelColor(i,_black);
    }
  }
  _strip->Show();
}

void StatusDisplay::setPixel(uint8_t pix, RgbColor clr) {
  _strip->SetPixelColor(pix,clr);
}

void StatusDisplay::show() {
  _strip->Show();
}
