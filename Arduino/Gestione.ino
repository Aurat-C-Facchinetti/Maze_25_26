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
// Motore A (sx)
#define PWMA 3
#define AIN1 4
#define AIN2 5

// Motore B (dx)
#define PWMB 6
#define BIN1 7
#define BIN2 8

int vel = 200; 

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000"; // holds received angle string (e.g., "090")
int val, cmTarget, tickTarget, encoder1Count;
char move;
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

#define signalA 19
#define signalB 18

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
  Serial.println(tickCount);
}
// delay
const unsigned long DUR_AVANTI_MS   = 3600;
const unsigned long DUR_INDIETRO_MS = 3600;
const unsigned long DUR_TURN_MS     = 3400;

void stopMotori() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void avanti() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, vel);
  analogWrite(PWMB, vel);
}

void indietro() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, vel);
  analogWrite(PWMB, vel);
}

void sinistra() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, vel);
  analogWrite(PWMB, vel);
}

void destra() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, vel);
  analogWrite(PWMB, vel);
}
#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) delay(10);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // Inizializza sensori sui rispettivi canali
  iniziaGyro(0);
  iniziaToFCorto(1);
  iniziaColore(2);
  iniziaToFLong(3);
  iniziaToFCorto(4);
  iniziaColore(5);
  iniziaToFLong(6);

  // encoder 

  pinMode(dirDx, OUTPUT);
  pinMode(dirSx, OUTPUT);
  pinMode(velDx, OUTPUT);
  pinMode(velSx, OUTPUT);

  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);


  delay(1000);

}

void loop() {
  
  if (Serial.available())
  {
    char chr = Serial.read();
    if(chr == 'g'){ // Orientation read
      idx = 10;
      val_idx=0;

      Serial.print("Yaw:");

      
    }
    if(chr == 'w') // Movement command
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    } else if(chr == 'k'){
      idx = -1;
      val_idx = 0;
    }
     else if(chr == 'l'){
      idx = -1;
      val_idx = 0;
    }
    else if(chr == 'W') //avanti forte
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    }
    else if(chr == 's') //indietro piano
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    }
    else if(chr == 'S')//indietro forte
    {
      idx =8;
      move=chr;
      val_idx = 0;
    }
    // Joint controls
    // base motor
    else if(chr == 'b')
    {
      idx = 0;
      val_idx = 0;
    }
    // shoulder motor
    else if(chr == 'v')
    {
      idx = 1;
      val_idx = 0;
    }
    // GOMITO motor
    else if(chr == 'c')
    {
      idx = 2;
      val_idx = 0;
    }
    // POLSO motor
    else if(chr == 'x')
    {
      idx = 3;
      val_idx = 0;
    }
    
    // MANO motor
    else if(chr == 'z')
    {
      idx = 4;
      val_idx = 0;
    }
    else if(chr == 'f')
    {
      idx = 7;
      val_idx = 0;
    }
    
    else if(chr == 'a')
    {
      idx = 5;
      val_idx = 0;
    }

    else if(chr == 'd')
    {
      idx = 6;
      val_idx = 0;
    }
    
    // Separator
    else if(chr == ',') {
      Serial.println(value);
      val = atoi(value); // Convert received number string to int
      
      Serial.flush();
      if(idx == 8)
      {
        switch(move){ // Movement based on selected command (forward/backward)
          case 'w':
            cmTarget=val;
            Serial.println("qwertyuio");
            Serial.println(val);
            tickTarget = 700 * cmTarget / 22;
            tickCount=0;
            Serial.println(tickTarget);
            avanti();
            while (true) {
              if (tickTarget < abs(tickCount)) {
              
                stopMotori();
                tickCount=0;
                break;
              }
            }
            break;
          case 's':
            cmTarget=val;
            tickTarget = 700 * cmTarget / 22;
            indietro();
            tickCount=0;
            while (true) {
              if (tickTarget < abs(tickCount)) {
                stopMotori();
                tickCount=0;
                break;
              }
            }
            break;      
        }       
      Serial.print("Yaw:");
      }
      // Process joint movement
      else if(idx == 0)
      {
        val = map(val, 0, 180, 180, 0);
      }
      else if(idx == 1)
      {
      }
      else if(idx == 2)
      { 
         val = map(val, 0, 180, 20, 157);
      }
      else if(idx == 3)
      { 
            }
      else if(idx == 4)
      {
         }
      else if(idx == 7)
      { 
      }
    
      else if(idx == 5)
      { 
       sinistra();
      Serial.print("Yaw:");
      
      }
      else if(idx == 6)
      {
       
       destra();
       
        Serial.print("Yaw:");
        
      }
      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    else // Store digits into value array
    {
      Serial.println(chr);
      value[val_idx] = chr;
      val_idx++;
    }
  }
  

  delay(100);
}
