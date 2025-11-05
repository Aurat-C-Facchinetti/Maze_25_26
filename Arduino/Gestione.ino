#pragma region LIBRERIE

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_TCS34725.h>

#pragma endregion

#pragma region COSTRUTTORI

Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_VL6180X  vl;
Adafruit_VL53L0X  lox;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#pragma endregion

#pragma region VARIABILI_GLOBALI

bool isMuro = false;
volatile long tickCount = 0;

#pragma endregion

/*
  CANALI
  0 -> Giroscopio (BNO055)
  1 -> ToF corto (VL6180X)
  2 -> Colore (TCS34725)
  3 -> ToF long (VL53L0X)
  4 -> ToF corto (VL6180X)
  5 -> Colore (TCS34725)
  6 -> ToF long (VL53L0X)
*/

#pragma region PIN_MOTORI

#define dirDx 7
#define dirSx 8
#define velDx 5   // PWM
#define velSx 6   // PWM

#pragma endregion

#pragma region PIN_ENCODERS

#define signalA 2
#define signalB 3

#pragma endregion

#pragma region UTILITY

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
  delay(2);
}

#pragma endregion

#pragma region GIROSCOPIO

bool iniziaGyro(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!bno.begin()) {
    Serial.print("Problema: BNO055 ");
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

void leggiGyro(uint8_t ch, float* yaw, float* pitch, float* roll) {
  TCA9548A(ch);
  sensors_event_t orient, accel;
  bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);
  if (yaw)   *yaw   = orient.orientation.x;
  if (pitch) *pitch = orient.orientation.y;
  if (roll)  *roll  = orient.orientation.z;
}

// (lasciate le tue helper, se vuoi usarle con un event già letto)
double getX(sensors_event_t* event) {
  double x = -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
  }
  return x;
}
double getY(sensors_event_t* event) {
  double y = -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    y = event->orientation.y;
  }
  return y;
}
double getZ(sensors_event_t* event) {
  double z = -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    z = event->orientation.z;
  }
  return z;
}

#pragma endregion

#pragma region TOF_CORTO

bool iniziaToFCorto(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!vl.begin()) {
    Serial.print(F("Problema: VL6180X "));
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

double leggiTofCorto(uint8_t ch, uint8_t* outStatus = nullptr) {
  TCA9548A(ch);

  double ris = -1.0;
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (outStatus) {
    *outStatus = status;
  }
  if (status == VL6180X_ERROR_NONE) {
    ris = range / 10.0;           // cm
    isMuro = (ris <= 10.0);       // soglia 10 cm
  } else {
    isMuro = false;
  }
  return ris;
}

#pragma endregion

#pragma region TOF_LONG

bool iniziaToFLong(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!lox.begin()) {
    Serial.print(F("Problema: VL53L0X "));
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

double leggiTofLong(uint8_t ch, uint8_t* outStatus = nullptr) {
  TCA9548A(ch);

  double ris = -1.0;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);

  uint8_t status = m.RangeStatus;
  if (outStatus) {
    *outStatus = status;
  }
  if (status != 4) {
    ris = m.RangeMilliMeter / 10.0; // cm
  }
  return ris;
}

#pragma endregion

#pragma region COLOR_SENSOR

bool iniziaColore(uint8_t ch) {
  bool iniz = true;
  TCA9548A(ch);
  if (!tcs.begin()) {
    Serial.print("Problema: TCS34725 ");
    Serial.println(ch);
    iniz = false;
  }
  return iniz;
}

// ritorna 'r','v','b','n' ; outStatus: 0 ok, 1 errore/nero (tutti 0)
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

  if (outStatus) *outStatus = st;
  if (r) *r = rr;
  if (g) *g = gg;
  if (b) *b = bb;
  if (c) *c = cc;

  return dom;
}

#pragma endregion

#pragma region OLD

void encoderReading() {
  int B = digitalRead(signalB);
  if (B == 1) {
    tickCount -= 1;
  } else {
    tickCount += 1;
  }
}

void go(int parCmGoal) {
  tickCount = 0;
  int targetTick = (488 /*forse*/ * parCmGoal) / 22;
  foward(100);
  while (abs(tickCount) < targetTick) {
    // waits
  }
  stopMotors();
}

void foward(int parVel) {
  digitalWrite(dirDx, HIGH);
  digitalWrite(dirSx, HIGH);
  analogWrite(velDx, parVel);
  analogWrite(velSx, parVel);
}

void stopMotors() {
  analogWrite(velDx, 0);
  analogWrite(velSx, 0);
}

void turnRight(int parVel) {
  digitalWrite(dirDx, LOW);
  digitalWrite(dirSx, HIGH);
  analogWrite(velDx, parVel);
  analogWrite(velSx, parVel);
}

void turnLeft(int parVel) {
  digitalWrite(dirDx, HIGH);
  digitalWrite(dirSx, LOW);
  analogWrite(velDx, parVel);
  analogWrite(velSx, parVel);
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) delay(10);

  // Inizializza sensori sui rispettivi canali
  iniziaGyro(0);
  iniziaToFCorto(1);
  iniziaColore(2);
  iniziaToFLong(3);
  iniziaToFCorto(4);
  iniziaColore(5);
  iniziaToFLong(6);

  // encoder (lasciati come nel tuo modello)
/*
  pinMode(dirDx, OUTPUT);
  pinMode(dirSx, OUTPUT);
  pinMode(velDx, OUTPUT);
  pinMode(velSx, OUTPUT);

  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);
*/

  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    int b = Serial.read();
    int cmd= b - '0'; // converte il char digitato in numero
    switch (cmd) {
      case 0: { // gyro
        float yaw = 0.0;
        float pitch = 0.0;
        float roll = 0.0;
        leggiGyro(0, &yaw, &pitch, &roll);
        Serial.print(yaw, 2); Serial.print(' ');
        Serial.print(pitch, 2); Serial.print(' ');
        Serial.println(roll, 2);
        Serial.println();
        break;
      }
      case 1: { // tof corto ch1
        uint8_t s;
        double d = leggiTofCorto(1, &s);
        Serial.println(d, 2);
        Serial.println();
        break;
      }
      case 2: { // colore ch2
        uint16_t r, g, b, c;
        uint8_t s;
        char dom = leggiColore(2, &r, &g, &b, &c, &s);
        Serial.println(dom);
        Serial.println();
        break;
      }
      case 3: { // tof lungo ch3
        uint8_t s;
        double d = leggiTofLong(3, &s);
        Serial.println(d, 2);
        Serial.println();
        break;
      }
      case 4: { // tof corto ch4
        uint8_t s;
        double d = leggiTofCorto(4, &s);
        Serial.println(d, 2);
        Serial.println();
        break;
      }
      case 5: { // colore ch5
        uint16_t r, g, b, c;
        uint8_t s;
        char dom = leggiColore(5, &r, &g, &b, &c, &s);
        Serial.println(dom);
        Serial.println();
        break;
      }
      case 6: { // tof lungo ch6
        uint8_t s;
        double d = leggiTofLong(6, &s);
        Serial.println(d, 2);
        Serial.println();
        break;
      }
      default: {
        break;
      }
    }
  }

  delay(100);
}
