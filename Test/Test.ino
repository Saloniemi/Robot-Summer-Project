#include <ESP32Servo.h>

Servo myServo;
int bitRes = 14;
int pulseWidth = 1500; // start at centre
float tau = 20000.0 / (1 << bitRes);  // compute time per tick
int currTick = round(1500 / tau); // find middle tick
int minTick = round(500/tau);
int maxTick = round(2500/tau);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  myServo.attach(32);
  myServo.setTimerWidth(bitRes);
  myServo.writeTicks(currTick); //sets servo approx. to middle position
  // myServo.writeMicroseconds(pulseWidth);
  // Serial.println("Use + and - keys to adjust pulse width");
  // Serial.println("Use f (fine +5µs) or c (coarse +50µs)");

}

// if bitRes = 14, try 4 or 5 ticks
// if bitRes = 12, try 1 or 2 ticks
int ticks = 4;

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
  //   pulseWidth = constrain(pulseWidth, 500, 2500);
  //   myServo.writeMicroseconds(pulseWidth);
  //   Serial.print("Pulse: "); Serial.print(pulseWidth); Serial.println(" µs");
  // }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') currTick += ticks*10;
    if (c == 'd') currTick -= ticks*10;
    if (c == '+') currTick += ticks*4;
    if (c == '-') currTick -= ticks*4;
    if (c == 'f') currTick += ticks;
    if (c == 'r') currTick -= ticks;
    currTick = constrain(currTick, minTick, maxTick);
    myServo.writeTicks(currTick);
    Serial.print("Ticks: "); Serial.print(currTick); Serial.print("Pulse: "); Serial.print(currTick*tau); Serial.println(" µs");
  } 
}
