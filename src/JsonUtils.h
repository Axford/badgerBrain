#ifndef JSON_UTILS_
#define JSON_UTILS_

#include <ArduinoJson.h>


int getJsonInt(JsonObject & obj, const __FlashStringHelper * name, float def);
float getJsonFloat(JsonObject & obj, const __FlashStringHelper * name, float def);
void getJsonUInt8Vector(uint8_t arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, uint8_t def[]);
void getJsonFloatVector(float arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, float def[]);
void getJsonIntVector(int arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, int def[]);
void getJsonBooleanVector(boolean arr[], uint8_t items, JsonObject & obj, const __FlashStringHelper * name, boolean def[]);

void printArray(const __FlashStringHelper * name, float arr[], uint8_t items);

void serializeFloatVector(float arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name);
void serializeUInt8Vector(uint8_t arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name);
void serializeIntVector(int arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name);
void serializeBooleanVector(boolean arr[], uint8_t items, JsonDocument & doc, const __FlashStringHelper * name);



#endif
