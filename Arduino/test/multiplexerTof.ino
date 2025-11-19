#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl;

const uint8_t TCA_ADDR = 0x70;
const uint8_t CH1 = 1; //TofCorto1
const uint8_t CH4 = 4; //TofCorto2


void TCA9548A(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(2);
}

bool iniziaToF(uint8_t ch) {
  bool iniz= true;
  TCA9548A(ch);
  
  if (!vl.begin()) {
    Serial.print(F("Problema: VL6180X "));
    Serial.println(ch);
    iniz= false;
  }
  return iniz;
}

double leggiTof(uint8_t ch, uint8_t* outStatus = nullptr) {
  TCA9548A(ch);

  double ris = -1.0;
  uint8_t range = vl.readRange();
  uint8_t status   = vl.readRangeStatus();

  if (outStatus) *outStatus = status;

  if (status == VL6180X_ERROR_NONE) {
    ris = range / 10.0;
  }

  return ris;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();

  iniziaToF(CH1);
  iniziaToF(CH4);
}

void loop() {
  uint8_t s1, s4;
  double d1 = leggiTof(CH1, &s1);
  double d4 = leggiTof(CH4, &s4);

  Serial.println(d1, 2);
  Serial.println(d4, 2);
  Serial.println();

  delay(500);
}
