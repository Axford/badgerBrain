#include "JsonUtils.h"


int getJsonInt(JsonObject & obj, const __FlashStringHelper * name, float def) {
  if (obj.containsKey(name)) {
    return obj[name];
  }  else
    return def;
}


float getJsonFloat(JsonObject & obj, const __FlashStringHelper * name, float def) {
  if (obj.containsKey(name)) {
    return obj[name];
  }  else
    return def;
}

void getJsonUInt8Vector(uint8_t arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, uint8_t def[]) {
  // pre-populate arr with defaults
  for (uint8_t i=0; i<items; i++) arr[i] = def[i];

  JsonArray arrObj = obj[name];
  uint8_t i=0;
  if (arrObj) {
    for (JsonVariant f : arrObj) {
      if (i < items) {
        arr[i] = f;
      } else
        break;

      i++;
    }
  }
}

void getJsonFloatVector(float arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, float def[]) {
  // pre-populate arr with defaults
  for (uint8_t i=0; i<items; i++) arr[i] = def[i];

  JsonArray arrObj = obj[name];
  uint8_t i=0;
  if (arrObj) {
    for (JsonVariant f : arrObj) {
      if (i < items) {
        arr[i] = f;
      } else
        break;

      i++;
    }
  }
}

void getJsonIntVector(int arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, int def[]) {
  // pre-populate arr with defaults
  for (uint8_t i=0; i<items; i++) arr[i] = def[i];

  JsonArray arrObj = obj[name];
  uint8_t i=0;
  if (arrObj) {
    for (JsonVariant f : arrObj) {
      if (i < items) {
        arr[i] = f;
      } else
        break;

      i++;
    }
  }
}

void getJsonBooleanVector(boolean arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, boolean def[]) {
  // pre-populate arr with defaults
  for (uint8_t i=0; i<items; i++) arr[i] = def[i];

  JsonArray arrObj = obj[name];
  uint8_t i=0;
  if (arrObj) {
    for (JsonVariant f : arrObj) {
      if (i < items) {
        arr[i] = f;
      } else
        break;

      i++;
    }
  }
}


void printArray(const __FlashStringHelper * name, float arr[], uint8_t items) {
  Serial.print(name);
  Serial.print(':');
  for (uint8_t i=0; i<items; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(arr[i]);
  }
  Serial.println();
}



void serializeFloatVector(float arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name) {
  JsonArray arrObj = doc.createNestedArray(name);
  for (uint8_t i=0; i<items; i++) {
    arrObj.add(arr[i]);
  }
}

void serializeUInt8Vector(uint8_t arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name) {
  JsonArray arrObj = doc.createNestedArray(name);
  for (uint8_t i=0; i<items; i++) {
    arrObj.add(arr[i]);
  }
}

void serializeIntVector(int arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name) {
  JsonArray arrObj = doc.createNestedArray(name);
  for (uint8_t i=0; i<items; i++) {
    arrObj.add(arr[i]);
  }
}

void serializeBooleanVector(boolean arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name) {
  JsonArray arrObj = doc.createNestedArray(name);
  for (uint8_t i=0; i<items; i++) {
    arrObj.add(arr[i]);
  }
}
