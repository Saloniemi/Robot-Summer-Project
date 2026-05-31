#include <ESP32Servo.h>

Servo myServo;
Servo myServoL;

int bitRes = 14;
int pulseWidth = 1500; // start at min
float tau = 20000.0 / (1 << bitRes);  // compute time per tick
int currTick = round(pulseWidth / tau); // find start tick
int minTick = round(0/tau);
int maxTick = round(3000/tau);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  myServo.attach(12, 500, 2400);
  myServo.setTimerWidth(bitRes);
  myServo.writeTicks(currTick); //sets servo approx. to middle position
  // myServo.writeMicroseconds(pulseWidth);
  // Serial.println("Use + and - keys to adjust pulse width");
  // Serial.println("Use f (fine +5µs) or c (coarse +50µs)");

}

// if bitRes = 14, try 4 or 5 ticks
// if bitRes = 12, try 1 or 2 ticks
int ticks = 2;

void loop() {
  // put your main code here, to run repeatedly:
  
  // if (Serial.available()) {
  //   char c = Serial.read();
  //   if (c == 'c') pulseWidth += 50;
  //   if (c == 'd') pulseWidth -= 50;
  //   if (c == '+') pulseWidth += 10;
  //   if (c == '-') pulseWidth -= 10;
  //   if (c == 'f') pulseWidth += 5;
  //   if (c == 'r') pulseWidth -= 5;
  //   pulseWidth = constrain(pulseWidth, 500,+5+2-+ 2500);
  //   myServo.writeMicroseconds(pulseWidth);
  //   Serial.print("Pulse: "); Serial.print(pulseWidth); Serial.println(" µs");
  // }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') currTick += ticks*20;
    if (c == 'd') currTick -= ticks*20;
    if (c == '+') currTick += ticks*5;
    if (c == '-') currTick -= ticks*5;
    if (c == 'f') currTick += ticks;
    if (c == 'r') currTick -= ticks;
    currTick = constrain(currTick, minTick, maxTick);
    myServo.writeTicks(currTick);
    Serial.print("Ticks: "); Serial.print(currTick); Serial.print("Pulse: "); Serial.print(currTick*tau); Serial.println(" µs");
  } 
}
