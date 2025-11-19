#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox;

const uint8_t TCA_ADDR = 0x70;
const uint8_t CH3 = 3; // ToF long 1
const uint8_t CH6 = 6; // ToF long 2

void TCA9548A(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(2);
}

bool iniziaToF(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!lox.begin()) {
    Serial.print(F("Problema: VL53L0X "));
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

double leggiTof(uint8_t ch, uint8_t* outStatus = nullptr) {
  TCA9548A(ch);

  double ris = -1.0;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);

  uint8_t status = m.RangeStatus;
  if (outStatus) *outStatus = status;

  if (status != 4) {                 // più permissivo: evita -1 “sopra i 6 mm”
    ris = m.RangeMilliMeter / 10.0;  // cm
  }
  return ris;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();

  iniziaToF(CH3);
  iniziaToF(CH6);
}

void loop() {
  uint8_t s3, s6;
  double d3 = leggiTof(CH3, &s3);
  double d6 = leggiTof(CH6, &s6);

  Serial.println(d3, 2);
  Serial.println(d6, 2);
  Serial.println();

  delay(500);
}
