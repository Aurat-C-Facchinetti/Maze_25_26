#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 500;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port open
  iniziaGyro();

  TCA9548A(0); // dice che vuole usare SDA SCl del canale 0
  delay(1000);
}

void loop() {
  sensors_event_t parOrentation, parAccell;
  bno.getEvent(&parOrentation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&parAccell, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  printEvent(&parOrentation);
  Serial.println();
  printEvent(&parAccell);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
  Serial.print(bus);
}

void iniziaGyro() {
  if (!bno.begin()) {
    Serial.print("Problema: BNO055");
    while (1);
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

