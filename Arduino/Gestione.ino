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

// Costruttore giroscopio
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

// Costruttore TOF
Adafruit_VL6180X vl = Adafruit_VL6180X();

// Costruttore TOF_LONG
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Costruttore Sensore colore
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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) delay(10);  // aspetta che la seraile si apra

  // inizializziamo i sensori
  iniziaGyro();
  iniziaToF();
  iniziaTOF_LONG();
  iniziaColorSensor();

  // encoder
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
  if (Serial.available()) {
    char chr = Serial.read();
    switch (chr) {
      case 'w':
        // esempio: vai avanti
        // foward(120);
        break;
    }
  }
}

#pragma region UTILITY

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

#pragma endregion

#pragma region GIROSCOPIO

void iniziaGyro() {
  if (!bno.begin()) {
    Serial.print("Problema: BNO055");
    while (1);
  }

}

double getX(sensors_event_t* event) {
  double x= -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
  }
  return x;
}

double getY(sensors_event_t* event) {
  double y= -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    y = event->orientation.y;
  }
  return y;
}

double getZ(sensors_event_t* event) {
  double z= -1;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    z = event->orientation.z;
  }
  return z;
}

#pragma endregion

#pragma region TOF

void iniziaToF() {
  if (!vl.begin()) {
    Serial.println(F("Problema: VL6180X"));
    while (1) delay(10); 
    
  }
}

double leggiTof() {
  uint8_t status;
  double range = vl.readRange();
  isMuro = (range <= 100);
  if (status >= VL6180X_ERROR_SYSERR_1 && status <= VL6180X_ERROR_SYSERR_5) {
    // da controlare il print
    Serial.println(status);
    range= -1;
  }
  return range;
}

#pragma endregion

#pragma region TOF_LONG

void iniziaTOF_LONG() {
  if (!lox.begin()) {
    Serial.println(F("Problema: VL53L0X"));
    while (1) delay(10);
  }
}

double readTOF_LONG() {
  double var;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);   // true per debug

  if (measure.RangeStatus != 4) {     // 4 = out of range/phase error
    var = measure.RangeMilliMeter;
  } else {
    var = -1;
  }
  return var;
}

#pragma endregion

#pragma region COLOR_SENSOR

void iniziaColorSensor() {
  if (!tcs.begin()) {
    Serial.println("Problema: Colore");
    while (1) delay(10);
  }
}

char readColorSensor() {
  uint16_t r, g, b, c;
  char colore = 'n';
  tcs.getRawData(&r, &g, &b, &c);

  if (r > g && r > b) {
    colore = 'r';
  } else if (g > r && g > b) {
    colore = 'g';
  } else if (b > r && b > g) {
    colore = 'b';
  } else {
    // Nessun colore dominante
    colore = 'x';
  }
  return colore;
}

#pragma endregion

#pragma region OLD

void encoderReading() {
  int B = digitalRead(signalB);
  tickCount += (B == 1) ? -1 : +1;
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