//pin motors
#define dirDx 0
#define dirSx 0
#define velDx 0
#define velSx 0
//pin encoders
#define signalA 0;
#define signalB 0;
volatile long tickCount = 0;

void setup() {
  pinMode(dirDx, OUTPUT);
  pinMode(dirSx, OUTPUT);
  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  attachInterrupt(digitalPinToInterrupt(signalA), encoderReading, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}
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