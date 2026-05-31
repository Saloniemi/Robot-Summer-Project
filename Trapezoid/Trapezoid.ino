#include <ESP32Servo.h>
#include <math.h>

#define GLOBAL_MAX_ANGLE 170
#define GLOBAL_MIN_ANGLE 10  

#define SG_MAX_ANGLE 180
#define SG_MIN_ANGLE 0

// Ticks of each motor that corresponds to the above angles
#define J1R_MAX 1990 // GLOBAL_MAX_ANGLE
#define J1R_MID 1272 // 90 degrees
#define J1R_MIN 537 // GLOBAL_MIN_ANGLE
// #define J1R_RESTING 2055 // Resting Angle
#define J1R_RESTING 1272 // Resting Angle

#define J1L_MAX 1977 // GLOBAL_MAX_ANGLE
#define J1L_MID 1248 // 90 degrees
#define J1L_MIN 526 // GLOBAL_MIN_ANGLE
// #define J1L_RESTING 407 // Resting Angle
#define J1L_RESTING 1248 // Resting Angle

#define J2_MAX 1915 // GLOBAL_MAX_ANGLE
#define J2_MID 1191 // 90 degrees
#define J2_MIN 481 // GLOBAL_MIN_ANGLE
#define J2_RESTING 2003 // Resting Angle

#define S3_MAX 1987 // GLOBAL_MAX_ANGLE 1973/1993
#define S3_MID 1199 // 90 degrees 1191/1207
#define S3_MIN 437// GLOBAL_MIN_ANGLE 425/449
#define S3_RESTING 1387 // Resting Angle

#define S4_MAX 1983// GLOBAL_MAX_ANGLE 1973/1993
#define S4_MID 1170 // 90 degrees 1167/1173
#define S4_MIN 413 // GLOBAL_MIN_ANGLE
#define S4_RESTING 1370 // Resting Angle

Servo J0;
Servo J1R;
Servo J1L;
Servo J2;
Servo S3;
Servo S4;
String inputBuffer = "";

int low_range = 90 - GLOBAL_MIN_ANGLE;
int high_range = GLOBAL_MAX_ANGLE - 90;

int sg_low_range = 90 - SG_MIN_ANGLE;
int sg_high_range = SG_MAX_ANGLE - 90;

struct Commands{
  int angle;
  char joint_select;
  bool valid = false;
};

int R_angle;
int L_angle;
int R_ticks;
int L_ticks;
// reused global variables for all other joints (FOR NOW)
int general_angle;
int general_ticks;

int last_3 = S3_RESTING;
int last_2 = J2_RESTING;
int last_R = J1R_RESTING;
int last_L = J1L_RESTING;

bool is_R_leading;
float R_scaling, L_scaling, leading_end_tick;

//Calculate microseconds per tick, used to convert angles to tick
int bitRes = 14; 
int sg_bitRes = 11;
// float tau = 20000.0 / (1 << bitRes);  // compute time per tick

int angle_to_tick(int angle, int min, int mid, int max, int min_angle, int max_angle); 
// TO-DO FOR TRAPEZOIDAL MOTION: measure hardware clockspeed and set appropriate dt
void trapezoidal_motion(Servo &motor, int start_tick, int end_tick, float v_max = 5, float acc = 0.1, float snap_threshold = 5.0, int dt = 1);
void dual_trapezoidal_motion(Servo &motorR, Servo &motorL, int Rstart_tick, int Rend_tick, int Lstart_tick, int Lend_tick, float v_max = 5, float acc = 0.1, float snap_threshold = 5.0, int dt = 1);
inline void motor_select(int Rstart_tick, int Rend_tick, int Lstart_tick, int Lend_tick, float &R_scale, float &L_scale, float &leading_end_tick, bool &is_R_leading);
inline void get_position(float &p, float &v, float end_tick, float v_max, float acc, float dt);
inline void update_tick(char identifier, Servo &motor, int &counter, int &curr_tick, int &last_tick, float v);
inline float sign(float x);

void setup() {
  // Set up
  Serial.begin(9600);
  // Attach pins. Valid: 2, 4, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
  // TO DO: CHANGE MAGIC NUMBERS
  S3.attach(12, 500, 2400);
  J2.attach(33, 500, 2500);
  J1R.attach(25, 500, 2500);
  J1L.attach(32, 500, 2500);

  S3.setTimerWidth(bitRes); // TO DO: RE-CALIBRATE WITH A BITRES OF 10-11. ALSO, FIND TRUE RESTING POSITION. 
  J2.setTimerWidth(bitRes);
  J1R.setTimerWidth(bitRes);
  J1L.setTimerWidth(bitRes);
  // TO DO: RAMPING SEEMS REDUNDANT HERE IF ALL ITS DOING IS GOING FROM RESTING TO RESTING 
  // WHEN GOING TO REST, JOINT 3 FIRST, THEN JOINT 1, LASTLY JOINT 2. DO NOT MESS THIS ORDER UP.
  trapezoidal_motion(S3, last_3, S3_RESTING, 5, 0.05, 10); 
  dual_trapezoidal_motion(J1R, J1L, last_R, J1R_RESTING, last_L, J1L_RESTING);
  trapezoidal_motion(J2, last_2, J2_RESTING);

  last_2 = J2_RESTING;
  last_R = J1R_RESTING;
  last_L = J1L_RESTING;
  last_3 = S3_RESTING;
}


// TO DO: ADD COMMAND OPTION FOR SNAPPING
void loop() {
  // Read incoming bytes without blocking the CPU
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    // Check for newline or carriage return (User pressed Enter)
    if (incomingChar == '\n' || incomingChar == '\r') {
      if (inputBuffer.length() > 0) {
        Commands newCommand;
        newCommand = processCommand(inputBuffer);

        // Check if command is valid
        if (!newCommand.valid) {
          // ignore command
          inputBuffer = "";
          continue;
        }
        
        //main motor control code
        // Separates into joints
        if (newCommand.joint_select == 'A'){ // Joint 1
          // Map from 90 degrees to 0 degrees (normal position), motors spin in opposite directions
          R_angle = 90 - newCommand.angle;
          L_angle = 90 + newCommand.angle;

          // Map from angles to ticks
          R_ticks = angle_to_tick(R_angle, J1R_MIN, J1R_MID, J1R_MAX, GLOBAL_MIN_ANGLE, GLOBAL_MAX_ANGLE);
          L_ticks = angle_to_tick(L_angle, J1L_MIN, J1L_MID, J1L_MAX, GLOBAL_MIN_ANGLE, GLOBAL_MAX_ANGLE);

          // Write to motors
          dual_trapezoidal_motion(J1R, J1L, last_R, R_ticks, last_L, L_ticks);

          // Update start position
          last_R = R_ticks;
          last_L = L_ticks;

          // Print for troubleshooting
          Serial.print("R Angle: "); Serial.print(R_angle);
          Serial.print(" R Ticks: "); Serial.println(R_ticks);
          Serial.print("L Angle: "); Serial.print(L_angle);
          Serial.print(" L Ticks: "); Serial.println(L_ticks);
          Serial.print("Leading Motor: "); Serial.print((is_R_leading) ? "R_motor" : "L_motor"); Serial.print(", L-scaling: "); Serial.print(L_scaling); Serial.print(", R-scaling: "); Serial.println(R_scaling);
          
        } else if (newCommand.joint_select == 'B'){ // Joint 2

          general_angle = 90 + newCommand.angle; // get angle from command
          general_ticks = angle_to_tick(general_angle, J2_MIN, J2_MID, J2_MAX, GLOBAL_MIN_ANGLE, GLOBAL_MAX_ANGLE); // converts degrees to ticks
          trapezoidal_motion(J2, last_2, general_ticks); // Perform single motor ramping
          last_2 = general_ticks; // Update new start position
          Serial.print("Angle: "); Serial.print(general_angle);
          Serial.print(" Ticks: "); Serial.println(general_ticks);

        } else if (newCommand.joint_select == 'C'){
          // TO DO: CHANGE TICK UPDATE LOGIC TO MATCH THE 20 US INTERNAL DEADBAND FOR SG90
          general_angle = 90 - newCommand.angle;
          general_ticks = angle_to_tick(general_angle, S3_MIN, S3_MID, S3_MAX, SG_MIN_ANGLE, SG_MAX_ANGLE); // converts degrees to ticks
          trapezoidal_motion(S3, last_3, general_ticks, 5, 0.05, 10.0);
          last_3 = general_ticks;
          Serial.print("Angle: "); Serial.print(general_angle);
          Serial.print(" Ticks: "); Serial.println(general_ticks);


        } else if (newCommand.joint_select == 'R'){

          trapezoidal_motion(S3, last_3, S3_RESTING, 5, 0.05, 10.0);
          dual_trapezoidal_motion(J1R, J1L, last_R, J1R_RESTING, last_L, J1L_RESTING);
          trapezoidal_motion(J2, last_2, J2_RESTING);

          last_3 = S3_RESTING;
          last_2 = J2_RESTING;
          last_R = J1R_RESTING;
          last_L = J1L_RESTING;
        }
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
Commands processCommand(String cmd) {
  Commands command;

  cmd.trim(); // Clean up accidental spaces

  // Check for broken inputs
  if (cmd.length() < 2){
    command.valid = false;
    return command;
  }
  
  command.joint_select = toupper(cmd.charAt(0)); // 'A' corresponds to joint 0, 'B' joint 1, 'C' joint 2...

  String angle_cmd = cmd.substring(1);

  // Extract the numerical microsecond value (Rest of the string)
  command.angle = angle_cmd.toInt();

  if (command.joint_select == 'C' || command.joint_select == 'D'){ // For SG90s, the range of motion is expanded to (0,180)
      if (command.angle < -sg_low_range || command.angle > sg_high_range) {
        Serial.print("ERROR: "); Serial.print(command.angle);
        Serial.print(" angle is outside defined limits: ("); Serial.print(-sg_low_range); Serial.print(","); Serial.print(sg_high_range); Serial.println(").");
        command.valid = false;
        return command; 
      } else {
        command.valid = true;
        return command;
      }
  }

  // Safety Constraint: Protect the physical internal gear stops
  if (command.angle < -low_range || command.angle > high_range) {
    Serial.print("ERROR: "); Serial.print(command.angle);
    Serial.print(" angle is outside defined limits: ("); Serial.print(-low_range); Serial.print(","); Serial.print(high_range); Serial.println(").");
    command.valid = false;
    return command; 
  } 

  command.valid = true;
  return command; 
}

/**
  * Maps angles from -90 to 90 (90 degrees is set as zero neutral position.) 
  * MAY BE UNNECESSARY?
  */
int amap(int angle){
  return map(angle, -90, 90, 0, 180);
}

/**
  * Maps angle to ticks (important!)
  * Since 90 degrees may not be perfectly centered, the mapping separates incoming angles into piecewise bins
  */
int angle_to_tick(int angle, int min, int mid, int max, int min_angle, int max_angle){
  if (angle > 90){
    return round(map(angle, 90, max_angle, mid, max)); // Map angles (90, MAX) to (mid, max) ticks
  } else if (angle < 90){
    return round(map(angle, min_angle, 90, min, mid)); // Map angles (MIN, 90) to (min, mid) ticks
  } else {
    return mid; // Map to mid if angle is 90
  }
}

/**
  * Carries out the motion ramping using trapezoid profile
  * Assumes for each loop, microcontroller spends 1 microseconds
  */
void trapezoidal_motion(Servo &motor, int start_tick, int end_tick, float v_max, float acc, float snap_threshold, int dt){
  // Pre-loop initializations
  float p = float(start_tick);
  float v = 0;
  float p_buffer;
  int last_written_tick = -1;
  int current_tick = -1;
  int counter = 0;

  //loop
  while (fabs(float(end_tick) - p) > snap_threshold) {
    // Write current position (when integer value changes)
    current_tick = round(p);
    if (current_tick != last_written_tick){
      counter += 1;
      if (counter % 3 == 0){
        motor.writeTicks(current_tick);
        Serial.println(current_tick);
      }
      last_written_tick = current_tick;
    }
    // Deceleration (overshoot) buffer
    p_buffer = 2 * dt * v + fabs(v) * v / (2 * acc); // 2 * v is an arbitrary buffer
    // Switches sign of velocity when overshoot
    v += sign(end_tick - p - p_buffer) * acc * dt;
    // Clamp velocity to v_max
    v = constrain(v, -v_max, v_max);
    // Increment p by velocity
    p += v * dt;
  }

  // Snap
  motor.writeTicks(end_tick);
}

/**
  * Carries out the motion ramping using trapezoid profile
  * Implements v_max and acceleration scaling to sync two motors
  * Can be expanded to control multiple motors later
  */
void dual_trapezoidal_motion(Servo &motorR, Servo &motorL, int Rstart_tick, int Rend_tick, int Lstart_tick, int Lend_tick, float v_max, float acc, float snap_threshold, int dt){
  // Pre-loop initializations (IS THERE A WAY TO MAKE THIS CLEANER?)
  float p_R = float(Rstart_tick);
  float p_L = float(Lstart_tick);
  float p_leading; 
  float v_R = 0.0f;
  float v_L = 0.0f;
  int R_counter = 0;
  int L_counter = 0;
  int R_last_tick = -1;
  int L_last_tick = -1;
  int R_current_tick = -1;
  int L_current_tick = -1;

  // Determines which is the leading motor, and update the scaling factors
  motor_select(Rstart_tick, Rend_tick, Lstart_tick, Lend_tick, R_scaling, L_scaling, leading_end_tick, is_R_leading);

  // Set the initial leading p
  p_leading = (is_R_leading) ? p_R : p_L;

  // Core update loop (A LIL MESSY)
  while (fabs(leading_end_tick - p_leading) > snap_threshold) { // loop should end when leading motor completes its journey
    // Write current position (when integer value changes)
    R_current_tick = round(p_R);
    L_current_tick = round(p_L);

    // Update logic for R motor 
    update_tick('R', motorR, R_counter, R_current_tick, R_last_tick, v_R);
    update_tick('L', motorL, L_counter, L_current_tick, L_last_tick, v_L);

    // Update position and velocity update for each motor
    get_position(p_R, v_R, Rend_tick, v_max * R_scaling, acc * R_scaling, dt);
    get_position(p_L, v_L, Lend_tick, v_max * L_scaling, acc * L_scaling, dt);

    // Update leading p
    p_leading = (is_R_leading) ? p_R : p_L;
  }

  // Snap
  motorR.writeTicks(Rend_tick);
  motorL.writeTicks(Lend_tick);
}

/**
  * Apply a scaling factor to the max velocity and acceleration to the lagging motor
  * Lagging: travelling a shorter distance
  * Leading: travelling a longer distance. This motor "leads" the motion.
  */
inline void motor_select(int Rstart_tick, int Rend_tick, int Lstart_tick, int Lend_tick, float &R_scale, float &L_scale, float &leading_end_tick, bool &is_R_leading){
  // Calculate distances two motors each need to travel
  float R_dist = abs(Rend_tick - Rstart_tick);
  float L_dist = abs(Lend_tick - Lstart_tick);

  // Handle edge-case to avoid division by zero if the arm doesn't need to move
  if (R_dist == 0 && L_dist == 0) {
    R_scale = 1.0; L_scale = 1.0;
    is_R_leading = true; leading_end_tick = Rend_tick;
    return;
  }

  // The motor moving a shorter distance is slowed down proportionally.
  if (R_dist >= L_dist) {
    R_scale = 1.0;
    L_scale = L_dist / R_dist; // Yields a fraction <= 1.0
    leading_end_tick = Rend_tick;
    is_R_leading = true;
  } else {
    L_scale = 1.0;
    R_scale = R_dist / L_dist; // Yields a fraction < 1.0
    leading_end_tick = Lend_tick;
    is_R_leading = false;
  }
}

/**
  * Calculates positional updates per microcontroller cycle
  */
inline void get_position(float &p, float &v, float end_tick, float v_max, float acc, float dt){
  // Deceleration (overshoot) buffer
    float p_buffer = 2 * dt * v + fabs(v) * v / (2 * acc); // 2 * v is an arbitrary buffer
    // Switches sign of velocity when overshoot
    v += sign(end_tick - p - p_buffer) * acc * dt;
    // Clamp velocity to v_max
    v = constrain(v, -v_max, v_max);
    // Increment p by velocity
    p += v * dt;
}

/**
  * Returns +1 if x is positive, -1 if negative, 0 if 0
  */
inline float sign(float x){
  return (x > 0) - (x < 0);
}

/**
  * Checks when tick integers have updated
  * Writes to motor every three updates
  * This is so that tedious Servo.writeTick() are not done every loop, which slows down calculations BY A LOT (LIKE A LOT)
  */
inline void update_tick(char identifier, Servo &motor, int &counter, int &curr_tick, int &last_tick, float v){
   if (curr_tick != last_tick){   // writes tick only every 3 tick updates
      counter += 1; // tracks number of updates
      if (counter % 3 == 0){
        motor.writeTicks(curr_tick);
        // Serial.print(identifier); Serial.print("ticks:"); Serial.println(curr_tick); // for plotting check
        Serial.print(identifier); Serial.print("velocity:"); Serial.println(v); // for plotting check
      }
      last_tick = curr_tick; // updates tick
    }
}
