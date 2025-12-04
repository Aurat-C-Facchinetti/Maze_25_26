
#pragma region LIBRERIE

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_TCS34725.h>
#include <math.h>

#pragma endregion

#pragma region COSTRUTTORI

//Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
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
bool isInvertito = false;
float gyroValues[3] = {0.0, 0.0, 0.0};

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
//Custom I2C Addresses
uint8_t LOX1_ADDRESS = 0x30;
uint8_t LOX2_ADDRESS = 0x31;
uint8_t LOX3_ADDRESS = 0x32;
uint8_t LOX4_ADDRESS = 0x33;
//Tof's Shutdown pins 
int SHT_LOX1 = 7;
int SHT_LOX2 = 9;
int SHT_LOX3 = 11;
int SHT_LOX4 = 13;
//Sensor objects
Adafruit_VL6180X lox1;
Adafruit_VL6180X lox2;
Adafruit_VL6180X lox3;
Adafruit_VL6180X lox4;
Adafruit_VL6180X* tofFrontShort;
Adafruit_VL6180X* tofBackShort;
Adafruit_VL6180X* tofLeftShort;
Adafruit_VL6180X* tofRightShort;


int CANALE_GYRO = 0;
int CANALE_TOF_1 = 1; //tof corto
int CANALE_COLORE_1 = 2;
int CANALE_TOF_2 = 3; //tof lungo
int CANALE_TOF_3 = 4; //tof corto
int CANALE_COLORE_2 = 5;
int CANALE_TOF_4 = 6; //tof lungo


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

#pragma region GIROSCOPIO_UPDATED

imu::Quaternion q0;
bool riferimentoImpostato = false;

static inline float rad2deg(float r) { return r * 180.0f / M_PI; }
static inline float clamp01(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);} 

void inziaGyro2() {
  if (!bno.begin()) {
    Serial.print("Problema: BNO055 ");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  riferimentoImpostato = false;
}
void leggiGyro2() {
  // Leggi quaternione attuale
  imu::Quaternion q = bno.getQuat();

  // Alla prima chiamata: salva come riferimento e basta
  if (!riferimentoImpostato) {
    q0 = q;
    riferimentoImpostato = true;
  }

  // Rotazione relativa: q_rel = inv(q0)*q = conj(q0)*q
  imu::Quaternion q_rel = q0.conjugate() * q;

  float w = q_rel.w();
  float x = q_rel.x();
  float y = q_rel.y();
  float z = q_rel.z();

  // roll (X)
  float roll = rad2deg(atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)));
  // pitch (Y)
  float s = 2.0f * (w * y - z * x);
  s = clamp01(s);
  float pitch = rad2deg(asinf(s));
  // yaw / heading (Z)
  float heading = rad2deg(atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)));

  // Normalizza heading a [-180, 180]
  if (heading < -180.0f) heading += 360.0f;
  if (heading >  180.0f) heading -= 360.0f;

  gyroValues[0] = heading;
  gyroValues[1] = roll;
  gyroValues[2] = pitch;
}

float getX() {
  return gyroValues[0];  // primo elemento
}

float getY() {
  return gyroValues[1];  // secondo elemento
}

float getZ() {
  return gyroValues[2];  // terzo elemento
}


#pragma endregion
#pragma region TOF_CORTO

void iniziaTofCorto(Adafruit_VL6180X &sensor, uint8_t shutPin, uint8_t newAddress) {
  if (!sensor.begin()) {
    Serial.print(F("Problema shut TOF: "));
    Serial.println(shutPin);
  }
  sensor.setAddress(newAddress);
}
void aggiornaMappaTof() {
  if (!isInvertito) {
    // robot normale
    tofFrontShort = &lox1;
    tofBackShort  = &lox3;
    tofLeftShort  = &lox2;
    tofRightShort = &lox4;
  } else {
    // robot girato: front <-> back, left <-> right
    tofFrontShort = &lox3;
    tofBackShort  = &lox1;
    tofLeftShort  = &lox4;
    tofRightShort = &lox2;
  }
}
void setIndirizzo() {
  /*if (isInvertito) {
        uint8_t temp;
        temp = LOX1_ADDRESS;
        LOX1_ADDRESS = LOX3_ADDRESS;
        LOX3_ADDRESS = temp;
        //scambio tof lunghi
        temp = LOX2_ADDRESS;
        LOX2_ADDRESS = LOX4_ADDRESS;
        LOX4_ADDRESS = temp;

        Adafruit_VL6180X temp2;
        temp2 = lox1;
        lox1 = lox3;
        lox3 = temp2;
        temp2 = lox2;
        lox2 = lox4;
        lox4 = temp2;

        int temp3;
        temp3 = SHT_LOX1;
        SHT_LOX1 = SHT_LOX3;
        SHT_LOX3 = temp3;
        temp3=SHT_LOX2;
        SHT_LOX2 = SHT_LOX4;
        SHT_LOX4 = temp3;
  } */
  // Tutti in reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);

  // Attivo i 4 TOF uno dopo l’altro
  Adafruit_VL6180X* sensori[4] = { &lox1, &lox2, &lox3, &lox4 };
  uint8_t shutdown[4] = { SHT_LOX1, SHT_LOX2, SHT_LOX3, SHT_LOX4 };
  uint8_t indirizzo[4] = { LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS, LOX4_ADDRESS };

  for (uint8_t i = 0; i < 4; i++) {
    if (isInvertito) 
    {
      i = (i + 2) % 4;
    }
    digitalWrite(shutdown[i], HIGH);
    delay(1000);
    iniziaTofCorto(*sensori[i], shutdown[i], indirizzo[i]);
    delay(1000);
  }
}

double leggiTofCorto(Adafruit_VL6180X &sensor) {
  int range  = sensor.readRange();
  int status = sensor.readRangeStatus();
  if (status == VL6180X_ERROR_RANGEOFLOW || status == VL6180X_ERROR_RANGEUFLOW || range > 180 || range == 0) {
    range = -1;
    isMuro=false;
  }else{
    double ris;
    ris = range / 10.0;           // cm
    isMuro = (ris <= 10.0);       // soglia 10 cm
  }
  return range;
}

#pragma endregion

#pragma region TOF_LONG

bool iniziaToFLong(uint8_t ch) {//STESSA COSA DEL COMMENTO ANTECEDENTE
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

bool iniziaColore(uint8_t ch) {//NEL CASO CAMBIA IN VOID
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

#pragma region MOTORI

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
void ruotaRelativa(int gradi, bool versoSinistra) {
  float start  = getX(); //x di ora
  // x di dove deve arrivare
  float target;
  if (versoSinistra) {
    target = start + gradi;
  } else {
    target = start - gradi;
  }

  if (target >  180.0f) target -= 360.0f;
  if (target < -180.0f) target += 360.0f;

  if (versoSinistra) {
    sinistra();
  }
  else{
    destra();
  }
  while (true) {
    leggiGyro2(); // aggiorna il gyro
    float current = getX();

    // errore angolare
    float err = target - current;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    if (versoSinistra) {
      if (err <= 0.0f) break;
    } else {
      if (err >= 0.0f) break;
    }
  }
  stopMotori();
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
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  setIndirizzo();
  // Inizializza sensori sui rispettivi canali
  //iniziaGyro(0);
  inziaGyro2();
  iniziaColore(2);
  iniziaColore(5);

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
      leggiGyro2();
      Serial.println(getX());
      Serial.println(getY());
      Serial.println(getZ());
    }
    if(chr == 'w') // Movement command
    {
      idx = 8;
      move=chr;
      val_idx = 0;
    } else if(chr == 'm'){
      idx = 1;
      val_idx = 0;
    }
    else if(chr == 's') //indietro 
    {
      idx = 2;
      move=chr;
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
    Serial.print("True");
      Serial.println(value);
      val = atoi(value); // Convert received number string to int
      
      Serial.flush();
      if(idx == 8)
      {
        switch(move){ // Movement based on selected command (forward/backward)
          case 'w':
            cmTarget=val;
            tickTarget = 700 * cmTarget / 22;
            tickCount=0;
            if(!isInvertito){
              avanti();
            }else{
              indietro();
            }
            while (true) {
              if (abs(tickCount) >= tickTarget) {
                stopMotori();
                tickCount = 0;
                break;
              } 
              // TORNA INDIETRO
               else {
                // leggi colore
                char colore = leggiColore(2, 0, 0, 0, 0);

                if (colore == 'n') {
                  long tickAndata = abs(tickCount);  // cm percorsi
                  stopMotori();
                  delay(50);

                  tickTarget = tickAndata;
                  tickCount= 0;
                  if(!isInvertito){
                    indietro();
                  }else{
                    avanti();
                  }
                  while (abs(tickCount) < tickTarget)
                  stopMotori();
                  tickCount = 0;
                  break;
                }
              } 
            }
            break;
          case 's':
            cmTarget=val;
            tickTarget = 700 * cmTarget / 22;
            if(!isInvertito){
              indietro();
            }else{
              avanti();
            }
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
      }
      else if(idx == 0) { //lettura dei colori DA PROVARE 
        uint16_t r, g, b, c;
        uint8_t s;
        if(val == 1){
          Serial.print("qwertyuiop");
          Serial.print(leggiColore(CANALE_COLORE_1, &r, &g, &b, &c, &s));
        }else{
          Serial.print("zxcvbnm");
          Serial.print(leggiColore(CANALE_COLORE_2, &r, &g, &b, &c, &s));
        }
      }
      else if(idx == 1){ //lettura Tof DA PROVARE
        uint8_t s;
        if(val == 1){
          Serial.print(leggiTofCorto(*tofFrontShort));
        }else if(val == 2){
          Serial.print(leggiTofCorto(*tofBackShort));
        }else if(val == 3){
         Serial.print(leggiTofCorto(*tofLeftShort));
        } else{
          Serial.print(leggiTofCorto(*tofRightShort));
        } 
      }else if(idx == 2){        
        isInvertito = !isInvertito;
        uint8_t temp;
        //scambio sensori colore
        temp = CANALE_COLORE_1;
        CANALE_COLORE_1 = CANALE_COLORE_2;
        CANALE_COLORE_2 = temp;
        //scambio tof corti
        aggiornaMappaTof();
      }
      else if(idx == 5)//SINISTRA
      { 
        ruotaRelativa(val, true);
      }
      else if(idx == 6)
      {
        ruotaRelativa(val, false);//DESTRA
      }
      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    else // Store digits into value array
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
  

  delay(100);
}
