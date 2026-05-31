#include <ESP32Servo.h>

#define GLOBAL_MAX_ANGLE 170
#define GLOBAL_MIN_ANGLE 10  

// Ticks of each motor that corresponds to the above angles
#define J1R_MAX 1990 // GLOBAL_MAX_ANGLE
#define J1R_MID 1272 // 90 degrees
#define J1R_MIN 537 // GLOBAL_MIN_ANGLE

#define J1L_MAX 1977 // GLOBAL_MAX_ANGLE
#define J1L_MID 1248 // 90 degrees
#define J1L_MIN 526 // GLOBAL_MIN_ANGLE

Servo J1R;
Servo J1L;
String inputBuffer = "";

int low_range = 90 - GLOBAL_MIN_ANGLE;
int high_range = GLOBAL_MAX_ANGLE - 90;


int R_angle;
int L_angle;
int R_ticks;
int L_ticks;

int angle_to_tick(int angle, int min, int mid, int max); 
int trapezoidal_motion(int start_tick, int end_tick, float v_max = 0.1, float acc = 0.01, float snap_threshold = 5.0; int dt = 1);

//Calculate microseconds per tick, used to convert angles to tick
int bitRes = 14;
float tau = 20000.0 / (1 << bitRes);  // compute time per tick
int middle = J1R_MID;

// Bit res is not set here, since the deadband of MG996R is around ~5 microseconds, which is larger than the default resolution

void setup() {
  // Set up
  Serial.begin(9600);
  J1R.attach(32, 500, 2500);
  J1L.attach(25, 500, 2500);
  J1R.setTimerWidth(bitRes);
  J1L.setTimerWidth(bitRes);
  J1R.writeTicks(middle); 
  J1L.writeTicks(middle);

}

void loop() {
  // Read incoming bytes without blocking the CPU
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    // Check for newline or carriage return (User pressed Enter)
    if (incomingChar == '\n' || incomingChar == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = ""; // Reset buffer for the next command
      }
    } else {
      // Append character to our command string
      inputBuffer += incomingChar;
    }
  }
}


/**
 * Parses the string buffer and updates the physical hardware registers
 */
void processCommand(String cmd) {
  cmd.trim(); // Clean up accidental spaces
  
  // Extract the numerical microsecond value (Rest of the string)
  int angle = cmd.toInt();

    // Safety Constraint: Protect the physical internal gear stops
  if (angle < -low_range || angle > high_range) {
    Serial.print("ERROR: "); Serial.print(angle);
    Serial.print(" angle is outside defined limits: ("); Serial.print(-low_range); Serial.print(","); Serial.print(high_range); Serial.println(").");
    return;
  }

  // Map from 90 degrees to 0 degrees (normal position), motors spin in opposite directions
  R_angle = 90 - angle;
  L_angle = 90 + angle;

  // Map from angles to ticks
  R_ticks = angle_to_tick(R_angle, J1R_MIN, J1R_MID, J1R_MAX);
  L_ticks = angle_to_tick(L_angle, J1L_MIN, J1L_MID, J1L_MAX);

  // Write to motors
  J1R.writeTicks(R_ticks); //reversed!!! 
  J1L.writeTicks(L_ticks);

  // Print for troubleshooting
  Serial.print("R Angle: "); Serial.print(R_angle);
  Serial.print(" R Ticks: "); Serial.println(R_ticks);
  Serial.print("L Angle: "); Serial.print(L_angle);
  Serial.print(" L Ticks: "); Serial.println(L_ticks); 
}

int amap(int angle){
  // Maps angles from -90 to 90 (90 degrees is set as zero neutral position.) **NOT NEEDED?**
  return map(angle, -90, 90, 0, 180);
}

int angle_to_tick(int angle, int min, int mid, int max){
  // Maps angle to ticks
  // Since 90 degrees may not be perfectly centered, the mapping separates incoming angles into piecewise bins
  if (angle > 90){
    return round(map(angle, 90, GLOBAL_MAX_ANGLE, mid, max)); // Map angles (90, MAX) to (mid, max) ticks
  } else if (angle < 90){
    return round(map(angle, GLOBAL_MIN_ANGLE, 90, min, mid)); // Map angles (MIN, 90) to (min, mid) ticks
  } else {
    return mid; // Map to mid if angle is 90
  }
}


