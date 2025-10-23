#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_TCS34725.h>

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

#pragma endregion

/*
  CANALI
  Dispatcher multiplexer TCA9548A per 7 canali (0..6)
  Invia sulla Serial (115200) il numero del canale e ottieni UNA lettura dal sensore giusto.

  Mappa canali:
  0 -> Giroscopio (BNO055)
  1 -> ToF corto (VL6180X)
  2 -> Colore (TCS34725)
  3 -> ToF long (VL53L0X)
  4 -> ToF corto (VL6180X)
  5 -> Colore (TCS34725)
  6 -> ToF long (VL53L0X)
*/

#pragma region PIN_MOTORI
//pin motors
#define dirDx 0
#define dirSx 0
#define velDx 0
#define velSx 0

#pragma endregion

#pragma region PIN_ENCODERS

//pin encoders
#define signalA 0;
#define signalB 0;
#pragma endregion

volatile long tickCount = 0;

void setup() {
  
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port open
  
  iniziaGyro();
  iniziaToF();
  iniziaTOF_LONG();
  iniziaColorSensor();
  
  delay(1000);

  /*
  pinMode(dirDx, OUTPUT);
  pinMode(dirSx, OUTPUT);
  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);
  */
}

void loop() {
  

  if (Serial.available())
  {
    char chr = Serial.read();
    switch(chr)
    {
      case 'w':

        break;
    }
    
  }

}

#pragma region UTILITY

// IMPOSTA IL CANALE SUL QUALE COMUNICARE : USEREMO SEMPRE QUESTO METODO QUANDO
// COMUNICHEREMO CON UN SENSORE.
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
  Serial.print(bus);
}

#pragma endregion

#pragma region GIROSCOPIO

// Inizializza il giroscopio
void iniziaGyro() {
  if (!bno.begin()) {
    Serial.print("Problema: BNO055");
    while (1);
  }
}

// Stampa gli eventi reativi al giroscopio
void printEvent(sensors_event_t* event) {
  double x = -1, y = -1 , z = -1; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
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

#pragma endregion 

#pragma region TOF

// Inizializza il TOF
void iniziaToF() {
  if (!vl.begin()) {
    Serial.println(F("Problema: VL6180X"));
    while (1) { delay(10); }
  }
} 

// Legge il TOF
double leggiTof() {
  double range = vl.readRange();
  
  if (range > 100) {
    isMuro= false;
  } else {
    isMuro= true;
  }
  return range;
}

// Stampa lo stato d'errore
void stampaStatoRange(uint8_t status) {
  if (status == VL6180X_ERROR_NONE) {
    Serial.println(F("Nessun errore"));
    return;
  }

  if (status >= VL6180X_ERROR_SYSERR_1 && status <= VL6180X_ERROR_SYSERR_5) {
    Serial.println(status);
    return;
  }
}

#pragma endregion

#pragma region TOF_LONG

// Inizializza il TOF_LONG
void iniziaTOF_LONG()
{
  Serial.println("Inizializzo VL53L0X...");
  if (!lox.begin()) {              // indirizzo default 0x29 sul canale selezionato
    Serial.println(F("Errore: VL53L0X non trovato sul canale selezionato"));
    while (1) delay(10);
  }

  Serial.println(F("VL53L0X pronto. Letture in mm:"));
}

double readTOF_LONG()
{
    double variabile;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);   // true per debug verbose

    if (measure.RangeStatus != 4) {     // 4 = fase non valida/out of range
      Serial.print("Distanza (mm): " + measure.RangeMilliMeter);
       variabile = measure.RangeMilliMeter;
    } else {
      Serial.println("Fuori range: " + measure.RangeMilliMeter);
       variabile = -1;
    }

    return variabile;
}


#pragma endregion

#pragma region COLOR_SENSOR

void iniziaColorSensor()
{
  if (!tcs.begin()) {
    Serial.println("Errore inizializzazione)");
    while (1) delay(10);
  }
  Serial.println("TCS34725 inizializzato");
}

char readColorSensor()
{
  uint16_t r, g, b, c;
  char colore = 'n';
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("🔴 R: "); Serial.print(r);
  Serial.print(" 🟢 G: "); Serial.print(g);
  Serial.print(" 🔵 B: "); Serial.print(b);
  Serial.print(" ⚪ C: "); Serial.println(c);

  if (r > g && r > b)
  {
    Serial.println("R");
    colore = 'r';

  } else if (g > r && g > b)
  {
    Serial.println("G");
    colore = 'g';

  } else if (b > r && b > g)
  {
    Serial.println("B");
    colore = 'b';
  } else
  {
    Serial.println("🎨 Nessun colore dominante 🤷‍♂️");
  }
  return colore;
}


#pragma endregion

#pragma region OLD



/// ------- ROBA PRECEDENTE --------- 
void encoderReading() {
  int B = digitalRead(signalB);
  tickCount += (B == 1) ? -1 : +1;
}

void go(int parCmGoal) {
  tickCount = 0;
  int targetTick = 488/*forse*/ * parCmGoal / 22;
  foward(100);
  while(abs (tickCount) < targetTick) {
    //waits 
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