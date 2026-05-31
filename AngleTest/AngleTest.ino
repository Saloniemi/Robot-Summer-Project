#include <ESP32Servo.h>

Servo J1L;
Servo J1R;

#define MIN_PULSE 544
#define MAX_PULSE 2400
#define J1L_START 544
#define J1L_END 2002
#define J1R_START 2002
#define J1R_END 544
#define BIT_RES 14

// motor directions
bool J1L_reverse = false;
bool J1R_reverse = true;

// Global variables
float tau;  
int L_start;
int R_start;
int distance;

// initialize position variables
int L_pos = 0;
int R_pos = 0;

void setup() {
  Serial.begin(9600);
  delay(500); // Give Serial time to stabilize

  // Convert pulsewidth to ticks
  tau = 20000.0 / (1 << BIT_RES);  // compute time per tick (~1.22 µs)
  L_start = round(J1L_START / tau);
  R_start = round(J1R_START / tau);
  distance = round(1000 / tau);

  // Attach and initialize bit resolution
  J1L.attach(25);
  J1L.setTimerWidth(BIT_RES);
  J1R.attach(32);
  J1R.setTimerWidth(BIT_RES);

  // Initialize servo positions
  J1L.writeTicks(L_start);
  J1R.writeTicks(R_start);
  
  Serial.println("System Initialized successfully");
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(3000);

  // If the motor is in reverse, distance is subtracted from start. Else it is added.
  if (J1L_reverse == true){
      L_pos = L_start - distance;
  } else {
      L_pos = L_start + distance;
  }

  if (J1R_reverse == true){
      R_pos = R_start - distance;
  } else {
      R_pos = R_start + distance;
  }

  J1L.writeTicks(L_pos);
  J1R.writeTicks(R_pos);
  Serial.print("J1L Ticks:"); Serial.println(L_pos); Serial.print("Pulse: "); Serial.print(L_pos*tau); Serial.println(" µs");
  Serial.print("J1R Ticks:"); Serial.println(R_pos); Serial.print("Pulse: "); Serial.print(R_pos*tau); Serial.println(" µs");

  delay(3000);

  L_pos = L_start;
  R_pos = R_start;

  J1L.writeTicks(L_pos);
  J1R.writeTicks(R_pos);
  Serial.print("J1L Ticks:"); Serial.println(L_pos); Serial.print("Pulse: "); Serial.print(L_pos*tau); Serial.println(" µs");
  Serial.print("J1R Ticks:"); Serial.println(R_pos); Serial.print("Pulse: "); Serial.print(R_pos*tau); Serial.println(" µs");


}

