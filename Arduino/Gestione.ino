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

#pragma region CONSTRUCTORS

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Adafruit_VL6180X lox1;
Adafruit_VL6180X lox2;
Adafruit_VL6180X lox3;
Adafruit_VL6180X lox4;
Adafruit_VL6180X lox5;
Adafruit_VL6180X lox6;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Servo myservo;

#pragma endregion

#pragma region GLOBAL_VARIABLES

int rifAngle = 0;
int velStart = 100;
int velMax = 240;
double drivePrevError = 0;

int ledVictim = 10;
int centerPos = 110;
int rightPos = 145;
int leftPos = 60;

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";
int val, cmTarget, tickTarget, encoder1Count;
bool isMuro = false;
volatile long tickCount = 0;
bool isInvertito = false;
float gyroValues[3] = { 0.0, 0.0, 0.0 };

// Custom I2C Addresses
uint8_t LOX1_ADDRESS = 0x30;
uint8_t LOX2_ADDRESS = 0x31;
uint8_t LOX3_ADDRESS = 0x32;
uint8_t LOX4_ADDRESS = 0x33;
uint8_t LOX5_ADDRESS = 0x34;
uint8_t LOX6_ADDRESS = 0x35;

// TOF shutdown pins
int SHT_LOX1 = 33;
int SHT_LOX2 = 32;
int SHT_LOX3 = 30;
int SHT_LOX4 = 5;
int SHT_LOX5 = 4;  // update with actual pin
int SHT_LOX6 = 3;  // update with actual pin

// TOF pointers: 1=front, 2=right-front, 3=right-back, 4=back, 5=left-back, 6=left-front
Adafruit_VL6180X* tofFront;
Adafruit_VL6180X* tofRightFront;
Adafruit_VL6180X* tofRightBack;
Adafruit_VL6180X* tofBack;
Adafruit_VL6180X* tofLeftBack;
Adafruit_VL6180X* tofLeftFront;

// Gyro reference quaternion
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

#pragma region MOTOR_AND_ENCODER_PINS

// Motor A (left)
#define PWMA 6
#define AIN1 23
#define AIN2 22

// Motor B (right)
#define PWMB 7
#define BIN1 24
#define BIN2 25

#define signalA 18
#define signalB 17

#pragma endregion

#pragma region GYRO

void iniziaGyro() {
  if (!bno.begin()) {
    Serial.println("Error: BNO055 (GYRO)");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  riferimentoImpostato = false;
}

void leggiGyro() {
  imu::Quaternion q = bno.getQuat();

  if (!riferimentoImpostato) {
    q0 = q;
    riferimentoImpostato = true;
  }

  // Relative rotation: q_rel = conj(q0) * q
  imu::Quaternion q_rel = q0.conjugate() * q;

  float w = q_rel.w();
  float x = q_rel.x();
  float y = q_rel.y();
  float z = q_rel.z();

  // Roll (X axis)
  float roll = rad2deg(atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)));
  // Pitch (Y axis)
  float s = 2.0f * (w * y - z * x);
  s = clamp01(s);
  float pitch = rad2deg(asinf(s));
  // Yaw / heading (Z axis)
  float heading = rad2deg(atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)));

  // Normalize heading to [-180, 180]
  if (heading < -180.0f) heading += 360.0f;
  if (heading > 180.0f) heading -= 360.0f;

  gyroValues[0] = heading;
  gyroValues[1] = roll;
  gyroValues[2] = pitch;
}

float getX() { return gyroValues[0]; }
float getY() { return gyroValues[1]; }
float getZ() { return gyroValues[2]; }

#pragma endregion

#pragma region TOF

void iniziaTof(Adafruit_VL6180X& sensor, uint8_t shutPin, uint8_t newAddress) {
  if (!sensor.begin()) {
    Serial.print(F("Error: TOF shutdown pin "));
    Serial.println(shutPin);
  }
  sensor.setAddress(newAddress);
}

void aggiornaMappaTof() {
  if (!isInvertito) {
    tofFront      = &lox1;
    tofRightFront = &lox2;
    tofRightBack  = &lox3;
    tofBack       = &lox4;
    tofLeftBack   = &lox5;
    tofLeftFront  = &lox6;
  } else {
    // Robot inverted: front <-> back, left-front <-> right-back, left-back <-> right-front
    tofFront      = &lox4;
    tofBack       = &lox1;
    tofRightFront = &lox5;
    tofRightBack  = &lox6;
    tofLeftBack   = &lox2;
    tofLeftFront  = &lox3;
  }
}

void setIndirizzo() {
  // Put all TOFs in reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
  delay(10);

  // Enable TOFs one by one and assign custom I2C addresses
  Adafruit_VL6180X* sensori[6] = { &lox1, &lox2, &lox3, &lox4, &lox5, &lox6 };
  uint8_t shutdown[6] = { SHT_LOX1, SHT_LOX2, SHT_LOX3, SHT_LOX4, SHT_LOX5, SHT_LOX6 };
  uint8_t indirizzo[6] = { LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS, LOX4_ADDRESS, LOX5_ADDRESS, LOX6_ADDRESS };

  for (uint8_t i = 0; i < 6; i++) {
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
    double ris = range / 10.0; // convert to cm
    isMuro = (ris <= 12.0);
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
  while (1);
}

#pragma endregion

#pragma region MOTORS

void encoderReading() {
  tickCount += 1;
}

void stopMotori() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void avanti(int parVelLeft, int parVelRight) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, parVelRight);
  analogWrite(PWMB, parVelLeft);
}

void indietro(int parVelLeft, int parVelRight) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, parVelLeft);
  analogWrite(PWMB, parVelRight);
}

void destra(int parvel) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, parvel);
  analogWrite(PWMB, parvel);
}

void sinistra(int parvel) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, parvel);
  analogWrite(PWMB, parvel);
}

void moveRobot(int parTargetTicks) {
  // Controls linear movement and handles the goBack flag
  goBack = false;
  leggiGyro();

  // --- PRE-MOVEMENT LATERAL TOF READING ---
  // Read the 4 lateral TOFs to calculate the robot's skew relative to the cell walls.
  // D_SENSORI = physical distance [mm] between the front and rear TOF on the same side.
  // Measure this on the physical robot and update accordingly.
  const float D_SENSORI = 40.0;

  float SL_F = leggiTof(*tofLeftFront);
  float SL_B = leggiTof(*tofLeftBack);
  float SR_F = leggiTof(*tofRightFront);
  float SR_B = leggiTof(*tofRightBack);

  bool hasSx = (SL_F > 0 && SL_B > 0);
  bool hasDx = (SR_F > 0 && SR_B > 0);

  double correctionAngle = 0.0;

  // Calculate skew angle using atan2 on the difference between front/rear readings on each side.
  // If both sides have valid walls, average the two skew estimates for better accuracy.
  if (hasSx && hasDx) { //formula to calculate the compensation for the default angle
    double skewL = atan2(SL_F - SL_B, D_SENSORI) * 180.0 / M_PI;
    double skewR = atan2(SR_B - SR_F, D_SENSORI) * 180.0 / M_PI;
    correctionAngle = (skewL + skewR) / 2.0;
  } else if (hasSx) {
    correctionAngle = atan2(SL_F - SL_B, D_SENSORI) * 180.0 / M_PI;
  } else if (hasDx) {
    correctionAngle = atan2(SR_B - SR_F, D_SENSORI) * 180.0 / M_PI;
  }
  // If no wall is visible on either side, correctionAngle stays 0 and movement proceeds normally

  // Apply skew correction to the reference angle so the robot drives aligned to the cell
  rifAngle += correctionAngle; //compensation formula

  // Move forward for the target number of ticks; stop early if goBack is triggered
  moveForTicks(parTargetTicks, rifAngle, true);

  if (goBack) {
    // Black tile detected: return to the last checkpoint
    long ticksToReturn = abs(tickCount);

    isInvertito = !isInvertito;                    // reverse direction
    moveForTicks(ticksToReturn, rifAngle, false);  // go back, ignore goBack flag
    isInvertito = !isInvertito;                    // restore original direction

    rifAngle -= correctionAngle; // restore reference angle after goBack

    Serial.println("-1"); // negative response to Raspberry: movement was interrupted
  } else {
    // Movement completed: remove the skew correction so the robot is re-centered in the cell
    rifAngle -= correctionAngle;

    Serial.println("1"); // positive response to Raspberry: movement completed successfully
  }
}

void moveForTicks(long parTargetTicks, double parAngoloRiferimento, bool checkGoBack) {
  // Moves until the target tick count is reached or goBack is triggered
  tickCount = 0;
  drivePrevError = 0; // reset derivative error for the straight-drive PID

  while (abs(tickCount) < parTargetTicks) {
    if (checkGoBack && goBack) { // exit loop if goBack is triggered and we are checking for it
      break;
    }
    driveStraightStep(parTargetTicks, parAngoloRiferimento); // PID step: adjust motor speeds to drive straight
  }

  stopMotori();
}

void driveStraightStep(long parTargetTicks, double parAngoloRiferimento) {
  double Kp = 75.0, Kd = 0.8;
  int velBase = calculateVelocity(parTargetTicks); // compute base speed with acceleration/deceleration profile

  leggiGyro();
  double error = angleError(parAngoloRiferimento, getX()); // heading error between target and current angle

  double derivative = error - drivePrevError;
  drivePrevError = error; // update previous error for next derivative calculation

  int correction = Kp * error + Kd * derivative;     // PID correction to apply to each side
  int maxCorrection = velBase * 0.33;                 // cap correction at 33% of base speed to avoid sharp turns
  correction = constrain(correction, -maxCorrection, maxCorrection);

  int velLeft  = constrain(velBase + correction, velStart, 255);
  int velRight = constrain(velBase - correction, velStart, 255);

  if (!isInvertito) {
    avanti(velLeft, velRight);
  } else {
    indietro(velLeft, velRight);
  }
}

int calculateVelocity(long parTicksTarget) {
  // Returns the target speed at the current tick position:
  // - acceleration ramp for the first 1/10 of the movement
  // - full speed in the middle
  // - deceleration ramp for the last 1/6 of the movement
  double accelFraction = 1.0 / 10.0, brakeFraction = 1.0 / 6.0;
  int accelBuffer = (int)round(parTicksTarget * accelFraction);
  int brakeBuffer = (int)round(parTicksTarget * brakeFraction);

  int velocity;
  if (abs(tickCount) < accelBuffer) {
    velocity = map((int)abs(tickCount), 0, accelBuffer, velStart, velMax);
  } else if ((parTicksTarget - abs(tickCount)) < brakeBuffer) {
    velocity = map((int)(parTicksTarget - abs(tickCount)), 0, brakeBuffer, velStart, velMax);
  } else {
    velocity = velMax;
  }

  return constrain(velocity, velStart, velMax);
}

double normalizeAngle(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

double angleError(double parTarget, double parCurrent) {
  return normalizeAngle(parTarget - parCurrent);
}

void rotate(double parTargetDelta) {
  double kp = 40.0, ki = 0.0, kd = 0.55, integral = 0.0;
  parTargetDelta = normalizeAngle(parTargetDelta);
  double prevError = parTargetDelta;

  leggiGyro();
  double startAngle = rifAngle;
  unsigned long lastTime = millis();
  bool isArrived = false;

  while (!isArrived) {
    leggiGyro();
    double currentAngle = getX();
    double currentDelta = normalizeAngle(currentAngle - startAngle);
    double error = parTargetDelta - currentDelta;

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
      outputPid = constrain(outputPid, -255.0, 255.0);

      if (outputPid > 0 && outputPid < 75)        outputPid = 75;
      else if (outputPid > -75 && outputPid < 0)  outputPid = -75;

      if (outputPid > 0) {
        sinistra((int)fabs(outputPid));
      } else {
        destra((int)fabs(outputPid));
      }
    }
  }
  Serial.println("1");
}

#pragma region VICTIMS

void blinkVictim() {
  // Blink the victim LED 5 times with 500ms on/off interval as required by the rules
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledVictim, HIGH);
    delay(500);
    digitalWrite(ledVictim, LOW);
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
    if (cmd == 'U' || cmd == 'u') {
      Serial.println("Victim: Unharmed");
    } else if (cmd == 'H' || cmd == 'h') {
      Serial.print("Victim: Harmed ");
      if (cmd == 'h') {
        Serial.println("left");
      } else {
        Serial.println("right");
      }
    } else if (cmd == 'S' || cmd == 's') {
      Serial.print("Victim: Stable ");
      if (cmd == 's') {
        Serial.println("left");
      } else {
        Serial.println("right");
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

  // Motor and encoder pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // TOF shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  pinMode(SHT_LOX5, OUTPUT);
  pinMode(SHT_LOX6, OUTPUT);
  aggiornaMappaTof();

  setIndirizzo();
  iniziaGyro();

  // Encoder interrupt
  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);

  // Reset button interrupt
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), reset, RISING);

  Serial.println();
  Serial.println("START");
}

void loop() {
  if (Serial.available()) {
    char chr = Serial.read();

    #pragma region IDX
    if (chr == 'g') {        // gyro X axis (heading)
      idx = 10;
      val_idx = 0;
    }
    if (chr == 'w') {        // movement command
      idx = 8;
      val_idx = 0;
    } else if (chr == 'm') { // TOF wall check
      idx = 1;
      val_idx = 0;
    } else if (chr == 's') { // invert robot direction
      idx = 2;
      val_idx = 0;
    } else if (chr == 'f') {
      idx = 7;
      val_idx = 0;
    } else if (chr == 'a') { // rotate left
      idx = 5;
      val_idx = 0;
    } else if (chr == 'd') { // rotate right
      idx = 6;
      val_idx = 0;
    } else if (chr == 'i') { // gyro Y axis (pitch/inclination)
      idx = 3;
      val_idx = 0;
    }
    #pragma endregion

    else if (chr == ',') {
      Serial.println("True");
      val = atoi(value);
      Serial.flush();

      if (idx == 8) {
        // Movement: value is distance in cm, converted to ticks
        cmTarget   = val;
        tickTarget = 1000 * cmTarget / 30;
        Serial.println(tickTarget);
        moveRobot(tickTarget);

      } else if (idx == 1) {
        // TOF reading: 1=front, 2=right-front, 3=right-back, 4=back, 5=left-back, 6=left-front
        if (val == 1) {
          leggiTof(*tofFront);
          Serial.println(isMuro);
        } else if (val == 2) {
          leggiTof(*tofRightFront);
          Serial.println(isMuro);
        } else if (val == 3) {
          leggiTof(*tofRightBack);
          Serial.println(isMuro);
        } else if (val == 4) {
          leggiTof(*tofBack);
          Serial.println(isMuro);
        } else if (val == 5) {
          leggiTof(*tofLeftBack);
          Serial.println(isMuro);
        } else if (val == 6) {
          leggiTof(*tofLeftFront);
          Serial.println(isMuro);
        }

      } else if (idx == 2) {
        // Invert robot direction and remap TOF pointers accordingly
        isInvertito = !isInvertito;
        aggiornaMappaTof();

      } else if (idx == 3) {
        // Read pitch (inclination) from gyro
        leggiGyro();
        Serial.println(getY());

      } else if (idx == 5) {
        // Rotate left
        if (!isInvertito) {
          Serial.println(rifAngle);
          rotate(val);
          rifAngle += 90;
        } else {
          Serial.println(rifAngle);
          rotate(-val);
          rifAngle -= 90;
        }

      } else if (idx == 6) {
        // Rotate right
        if (!isInvertito) {
          Serial.println(rifAngle);
          rotate(-val);
          rifAngle -= 90;
        } else {
          Serial.println(rifAngle);
          rotate(val);
          rifAngle += 90;
        }

      } else if (idx == 10) {
        // Read heading (yaw) from gyro
        leggiGyro();
        Serial.println(getX());
      }

      // Reset value buffer
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';

    } else {
      // Store incoming digit characters into value buffer
      value[val_idx] = chr;
      val_idx++;
    }
  }
  delay(100);
}
