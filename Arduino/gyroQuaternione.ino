#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// Sensore con indirizzo 0x29 (ADR a VDD)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Riferimento iniziale (quaternion)
imu::Quaternion q0;
bool riferimentoImpostato = false;

static inline float rad2deg(float r) { return r * 180.0f / M_PI; }
static inline float clamp01(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("BNO055 non trovato! Controlla i fili.");
    while (1);
  }

  delay(1000);                 // Tempo per stabilizzare
  bno.setExtCrystalUse(true);  // Migliora la precisione
  Serial.println("Pronto! Tieni fermo il sensore per impostare lo zero...");
}

void loop() {
  // Reset riferimento se arriva 'Z' dalla seriale
  if (Serial.available()) {
    if (toupper(Serial.read()) == 'Z') {
      riferimentoImpostato = false;
      Serial.println("Zero reset! Rimetti fermo il sensore...");
    }
  }

  // Leggi quaternione attuale
  imu::Quaternion q = bno.getQuat();

  // Alla prima lettura utile: salva come riferimento
  if (!riferimentoImpostato) {
    q0 = q;
    riferimentoImpostato = true;
    Serial.println("Riferimento impostato!");
    delay(300);
    return; // salta una stampa per pulizia
  }

  // Rotazione relativa: q_rel = inv(q0)*q = conj(q0)*q (per quaternioni unitari)
  imu::Quaternion q_rel = q0.conjugate() * q;

  float w = q_rel.w();
  float x = q_rel.x();
  float y = q_rel.y();
  float z = q_rel.z();

  // Converti a Euler ZYX (gradi), tilt-compensated
  // roll (X)
  float roll = rad2deg(atan2f(2.0f * (w * x + y * z),
                              1.0f - 2.0f * (x * x + y * y)));
  // pitch (Y)
  float s = 2.0f * (w * y - z * x);
  s = clamp01(s);
  float pitch = rad2deg(asinf(s));
  // yaw/heading (Z)
  float heading = rad2deg(atan2f(2.0f * (w * z + x * y),
                                 1.0f - 2.0f * (y * y + z * z)));

  // Normalizza heading a [-180, 180]
  if (heading < -180.0f) heading += 360.0f;
  if (heading >  180.0f) heading -= 360.0f;

  // Invia via seriale CSV: heading,roll,pitch (una cifra decimale)
  Serial.print(heading, 1);
  Serial.print(",");
  Serial.print(roll, 1);
  Serial.print(",");
  Serial.println(pitch, 1);

  delay(50); // ~20 FPS
}