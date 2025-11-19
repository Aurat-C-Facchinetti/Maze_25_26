#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const uint8_t TCA_ADDR = 0x70;
const uint8_t CH2 = 2;
const uint8_t CH5 = 5;

void TCA9548A(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(2);
}

bool iniziaColore(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!tcs.begin()) {
    Serial.print(F("Problema: TCS34725 "));
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

// Ritorna 'r','v','b','n' (outStatus: 0 ok, 1 errore/nero)
char leggiColore(uint8_t ch, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c, uint8_t* outStatus = nullptr) {
  TCA9548A(ch);

  uint16_t rr = 0;
  uint16_t gg = 0;
  uint16_t bb = 0;
  uint16_t cc = 0;

  tcs.getRawData(&rr, &gg, &bb, &cc);

  uint8_t st = 0;
  if (rr == 0 && gg == 0 && bb == 0 && cc == 0) {
    st = 1;
  } else {
    st = 0;
  }

  char dom = 'n';
  if (st == 0) {
    if (rr > gg && rr > bb) {
      dom = 'r';
    } else if (gg > rr && gg > bb) {
      dom = 'v';
    } else if (bb > rr && bb > gg) {
      dom = 'b';
    } else {
      dom = 'n';
    }
  }

  if (outStatus) {
    *outStatus = st;
  }
  if (r) *r = rr;
  if (g) *g = gg;
  if (b) *b = bb;
  if (c) *c = cc;

  return dom;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();

  iniziaColore(CH2);
  iniziaColore(CH5);
}

void loop() {
  uint16_t r2, g2, b2, c2;
  uint16_t r5, g5, b5, c5;
  uint8_t s2, s5;

  char d2 = leggiColore(CH2, &r2, &g2, &b2, &c2, &s2);
  char d5 = leggiColore(CH5, &r5, &g5, &b5, &c5, &s5);

  Serial.println(d2);
  Serial.println(d5);
  Serial.println();

  delay(500);
}
