#pragma region LIBRERIE

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <Servo.h>
#include <avr/wdt.h>

#pragma endregion

#pragma region COSTRUTTORI

//Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//Sensor objects
Adafruit_VL6180X lox1;
Adafruit_VL6180X lox2;
Adafruit_VL6180X lox3;
Adafruit_VL6180X lox4;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Servo myservo;

#pragma endregion

#pragma region VARIABILI_GLOBALI

int velStart = 100;
int velMax = 240;
double drivePrevError = 0; //variable for the derivative component of the PID for linear movement

//Sandra
int ledVictim = 10;
int centerPos = 110;
int rightPos = 145;
int leftPos = 60;

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";  // holds received angle string (e.g., "090")
int val, cmTarget, tickTarget, encoder1Count;
bool isMuro = false;
volatile long tickCount = 0;
bool isInvertito = false;
float gyroValues[3] = { 0.0, 0.0, 0.0 };

//Custom I2C Addresses
uint8_t LOX1_ADDRESS = 0x30; //0x30 with the pcb
uint8_t LOX2_ADDRESS = 0x31; //0x31 with the pcb
uint8_t LOX3_ADDRESS = 0x32; //0x32 with the pcb
uint8_t LOX4_ADDRESS = 0x33; //0x33 with the pcb
//Tof's Shutdown pins
int SHT_LOX1 = 33; //9 with the pcb
int SHT_LOX2 = 32; //11 with the pcb
int SHT_LOX3 = 30; //13 with the pcb
int SHT_LOX4 = 5; //15 with the pcb

Adafruit_VL6180X* tofFrontShort;
Adafruit_VL6180X* tofBackShort;
Adafruit_VL6180X* tofLeftShort;
Adafruit_VL6180X* tofRightShort;

//gyro
imu::Quaternion q0;
bool riferimentoImpostato = false;

static inline float rad2deg(float r) {
  return r * 180.0f / M_PI;
}
static inline float clamp01(float v) {
  return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
}

volatile bool goBack = false;

#pragma endregion

#pragma region PIN_MOTORI_AND_ENCODERS

// Motore A (sx)
#define PWMA 6
#define AIN1 23
#define AIN2 22

// Motore B (dx)
#define PWMB 7
#define BIN1 24
#define BIN2 25

#define signalA 18
#define signalB 17

#pragma endregion

#pragma region GYRO

void iniziaGyro() {
  if (!bno.begin()) {
    Serial.println("Problema: BNO055 (GYRO)");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  riferimentoImpostato = false;
}

void leggiGyro() {
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
  if (heading > 180.0f) heading -= 360.0f;

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

#pragma region TOF

void iniziaTof(Adafruit_VL6180X& sensor, uint8_t shutPin, uint8_t newAddress) {
  if (!sensor.begin()) {
    Serial.print(F("Problema shut TOF: "));
    Serial.println(shutPin);
  }
  sensor.setAddress(newAddress);
}

void aggiornaMappaTof() {
  if (!isInvertito) {
    // robot normale
    tofFrontShort = &lox1; //1 with the pcb
    tofBackShort = &lox3; //3 with the pcb
    tofLeftShort = &lox4; //4 with the pcb
    tofRightShort = &lox2; //2 with the pcb
  } else {
    // robot girato: front <-> back, left <-> right
    tofFrontShort = &lox3;
    tofBackShort = &lox1;
    tofLeftShort = &lox2;
    tofRightShort = &lox4;
  }
}
void setIndirizzo() {
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
    /*if (isInvertito) 
    {
      i = (i + 2) % 4;
    }*/
    digitalWrite(shutdown[i], HIGH);
    delay(10);
    iniziaTof(*sensori[i], shutdown[i], indirizzo[i]);
    delay(10);
  }
}

double leggiTof(Adafruit_VL6180X& sensor) {
  int range = sensor.readRange();
  int status = sensor.readRangeStatus();
  if (status == VL6180X_ERROR_RANGEOFLOW || status == VL6180X_ERROR_RANGEUFLOW || range > 180 || range == 0) {
    range = -1;
    isMuro = false;
  } else {
    double ris;
    ris = range / 10.0;      // cm
    isMuro = (ris <= 12.0);  // soglia 10 cm
  }
  return range;
}

#pragma endregion

#pragma region INTERRUPTS

void isBlack() {
  Serial.println("GOING BACK, black detected");
  goBack = true;
}

void reset() {
  PORTH &= ~(1 << PH3);
  PORTH &= ~(1 << PH4);

  wdt_enable(WDTO_15MS);

  while(1);
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
  //Serial.println(tickCount);
  //if (tickCount == 350) { //used to simulate the interrupt from the color sensor/raspy
    //goBack = true;
  //}
}

void stopMotori() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void avanti(int parVelLeft, int parVelRight) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, parVelRight);
  analogWrite(PWMB, parVelLeft);
}

void indietro(int parVelLeft, int parVelRight) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, parVelLeft);
  analogWrite(PWMB, parVelRight);
}

void sinistra(int parvel) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, parvel);
  analogWrite(PWMB, parvel);
}

void destra(int parvel) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, parvel);
  analogWrite(PWMB, parvel);
}

void moveRobot(int parTargetTicks) { //function that controls the linear movemnt of the robot (it handles the goBack flag activation)
  goBack = false;
  leggiGyro();
  double angoloRiferimento = getX();

  moveForTicks(parTargetTicks, angoloRiferimento, true); //true -> check if goBack goes to true and, if it does, return at the starting point

  if (goBack) { //if it needs to return at the starting point
    long ticksToReturn = abs(tickCount);

    isInvertito = !isInvertito; //in order to go backwards
    moveForTicks(ticksToReturn, angoloRiferimento, false); //false -> ignore goBack flag (it's already returning at the starting position)
    isInvertito = !isInvertito; //goes back to normal direction

    Serial.println("-1"); //he returned at the starting point -> negative movement response to raspy
  } else {
    Serial.println("1"); //movement completed correctly -> positive movement response to raspy
  }
}

void moveForTicks(long parTargetTicks, double parAngoloRiferimento, bool checkGoBack) { //function that continues to move untill it reaches the target or some black is detected (goBack flag setted to true)
  tickCount = 0;
  drivePrevError = 0; //reset the previous error for the derivative part of the PID

  while(abs(tickCount) < parTargetTicks) {
    if (checkGoBack && goBack) { //if he needs to check goBack and goBack is actually true -> exit the loop and stop the motors
      break;
    }
    driveStraightStep(parTargetTicks, parAngoloRiferimento); //uses PID to move the motors at the right velocity (also keep the robot straight)
    checkSerial();
  }

  stopMotori();
}

void driveStraightStep(long parTargetTicks, double parAngoloRiferimento) {
  double Kp = 5.0, Kd = 0.5;
  int velBase = calculateVelocity(parTargetTicks); //calculate the base velocity th robot has to go (acc./max vel./dec.)

  leggiGyro();
  double error = angleError(parAngoloRiferimento, getX()); //calcutates the error between the actual heading and the ref. heading

  double derivative = error - drivePrevError;
  drivePrevError = error; //update the previuos error

  int correction = Kp * error + Kd * derivative; //calculate the correction he needs to apply at the velocity on each side in order to compensate the error
  int maxCorrection = velBase * 0.33; //max 33% of correction, so it avoid big turns
  correction = constrain(correction, -maxCorrection, maxCorrection); //keeps the correction in a certain range of values (min - corr. - max)

  int velLeft = constrain(velBase - correction, velStart, 255); //keeps the velocity over the minimum velocity to move and the max velocity
  int velRight = constrain(velBase + correction, velStart, 255);;

  if (!isInvertito) {
    avanti(velLeft, velRight);
  } else {
    indietro(velLeft, velRight);
  }
}

int calculateVelocity(long parTicksTarget) {
  int velocity;
  double bufferFraction = 1.0 / 9.0;  //change it in order to have smoother accelerations/brakes (how long are the acceleration and brake parts)
  int buffer = (int)round(parTicksTarget * bufferFraction);

  if (abs(tickCount) < buffer) { //acceleration part (first ticks)
    velocity = map((int)abs(tickCount), 0, buffer, velStart, velMax); //convert tickCount from its range (0 - tick of the buffer portion) to another one (velStart - velMax)
  } else if ((parTicksTarget - abs(tickCount)) < buffer) { //brake part (last ticks)
    velocity = map((int)(parTicksTarget - abs(tickCount)), 0, buffer, velStart, velMax); 
  } else { //middle part (max velocity)
    velocity = velMax;
  }

  velocity = constrain(velocity, velStart, velMax); //keeps the velocity over the minimum velocity to move and the max velocity

  return velocity;
}

double normalizeAngle(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

double angleError(double parTarget, double parCurrent) {
  double error = parTarget - parCurrent;
  error = normalizeAngle(parTarget - parCurrent);
  return error;
}

void rotate(double parTargetDelta) {
  double kp = 3.0, ki = 0.0, kd = 0.05, integral = 0.0;
  parTargetDelta = normalizeAngle(parTargetDelta);
  double prevError = parTargetDelta;
  leggiGyro();
  double startAngle = getX();
  unsigned long lastTime = millis();

  bool isArrived = false;
  while (!isArrived) {
    checkSerial();
    leggiGyro();
    double currentAngle = getX();
    double currentDelta = normalizeAngle(currentAngle - startAngle);
    double error = parTargetDelta - currentDelta;
    //Serial.println(error);

    if (fabs(error) < 0.05) {      
      stopMotori();
      isArrived = true;
    } else {
      unsigned long now = millis();
      float secondsPassed = (now - lastTime) / 1000.0;
      lastTime = now;

      integral += error * secondsPassed;
      double derivative = (error - prevError) / secondsPassed;
      prevError = error;
      double outputPid = kp * error + ki * integral + kd * derivative;

      if (outputPid > 255) {
        outputPid = 255;
      } else if (outputPid < -255) {
        outputPid = -255;
      }

      if (outputPid > 0 && outputPid < 155) {
        outputPid = 155;
      } else if (outputPid > -155 && outputPid < 0) {
        outputPid = -155;
      }

      if (outputPid > 0) {
        sinistra((int)fabs(outputPid));
      } else {
        destra((int)fabs(outputPid));
      }
    }
  }
  Serial.println("1");
}

#pragma endregion

#pragma region VITTIME

void blinkVictim() {
  for ( int i = 0; i < 5; i++) { //it repeat for five times and alternate by a half second beetween on/off
    digitalWrite( ledVictim, HIGH);
    delay(500);
    digitalWrite( ledVictim, LOW);
    delay(500);
  }
}

void shootRight() {
  myservo.write(rightPos);
  delay(300);
  myservo.write(centerPos);
}

void shootLeft() {
  myservo.write(leftPos);
  delay(300);
  myservo.write(centerPos);
}

void checkSerial() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    stopMotori();
    //blinkVictim();
    if ( cmd == 'U' || cmd == 'u' ){
      Serial.println("Victim: Unharmed ");
    } else if ( cmd == 'H' || cmd == 'h' ){
      Serial.print("Victim: Harmed ");
      if (cmd == 'h') {
        Serial.println("left");
        //shootLeft();
        //shootLeft();
      } else { //it's not lowercase, so it's uppercase (right)
        Serial.println("right");
        //shootRight();
        //shootRight();
      }
    } else if ( cmd == 'S' || cmd == 's' ){
      Serial.print("Victim: Stable ");
      if (cmd == 's') {
        Serial.println("left");
        //shootLeft();
      } else { //it's not lowercase, so it's uppercase (right)
        Serial.println("right");
        //shootRight();
      }
    }
    delay(1500);
  }
}

#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) delay(10);

  //setting motors and encorder's pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //setting tofs, gyro and color sensors
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  aggiornaMappaTof();

  setIndirizzo();

  iniziaGyro();

  // encoder
  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);

  //interrupt for black tile
  //pinMode(19, INPUT);
  //attachInterrupt(digitalPinToInterrupt(19), isBlack, RISING);
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), reset, RISING);

  Serial.println();
  Serial.println("START");
}

void loop() {
  if (Serial.available()) {
    char chr = Serial.read();
    #pragma region IDX
    if (chr == 'g') {  //x axis on the gyro
      idx = 10;
      val_idx = 0;
    }
    if (chr == 'w')  // Movement command
    {
      idx = 8;
      val_idx = 0;
    } else if (chr == 'm') {  //wall check
      idx = 1;
      val_idx = 0;
    } else if (chr == 's')  //change direction
    {
      idx = 2;
      val_idx = 0;
    } else if (chr == 'f') {
      idx = 7;
      val_idx = 0;
    } else if (chr == 'a') {
      idx = 5;
      val_idx = 0;
    } else if (chr == 'd') {
      idx = 6;
      val_idx = 0;
    } else if (chr == 'i') { //y axis from the gyro (inclination)
      idx = 3;
      val_idx = 0;
    }

    #pragma endregion

    // Separator
    else if (chr == ',') {
      Serial.println("True");
      val = atoi(value);  // Convert received number string to int
      Serial.flush();
      if (idx == 8) {
        cmTarget = val;
        tickTarget = 1000 * cmTarget / 30;
        Serial.println(tickTarget);
        moveRobot(tickTarget);
      } else if (idx == 1) {  //lettura Tof
        if (val == 1) {
          //Serial.println(leggiTof(*tofFrontShort));
          leggiTof(*tofFrontShort);
          Serial.println(isMuro);
        } else if (val == 2) {
          //Serial.println(leggiTof(*tofRightShort));
          leggiTof(*tofRightShort);
          Serial.println(isMuro);
        } else if (val == 3) { //back
          //Serial.println(leggiTof(*tofLeftShort));
          leggiTof(*tofLeftShort);
          Serial.println(isMuro);
        } else { //left
          //Serial.println(leggiTof(*tofBackShort));
          leggiTof(*tofBackShort);
          Serial.println(isMuro);
        }
      } else if (idx == 2) {
        isInvertito = !isInvertito;
        //scambio tof corti
        aggiornaMappaTof();
      } else if (idx == 3) {
        leggiGyro();
        Serial.println(getY());
      } else if (idx == 5)  //SINISTRA
      {
        if (!isInvertito) {
          rotate(val);
        } else {
          rotate(-val);
        }
      } else if (idx == 6) {
        if (!isInvertito) {
          rotate(-val);
        } else {
          rotate(val);
        }
      } else if (idx == 10) {
        leggiGyro();
        Serial.println(getX());
      }
      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    } else  // Store digits into value array
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
  delay(100);
}
