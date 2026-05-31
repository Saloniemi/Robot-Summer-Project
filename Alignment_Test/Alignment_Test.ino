#include <ESP32Servo.h>

Servo J1R;
Servo J1L;

int bitRes = 14;
int pulseWidthR = 2002; // start at min
int pulseWidthL = 544;
float tau = 20000.0 / (1 << bitRes);  // compute time per tick
int currTickR = round(pulseWidthR / tau); // find start tick
int currTickL = round(pulseWidthL / tau); // find start tick
int minTick = round(544/tau);
int maxTick = round(2400/tau);

String inputBuffer = "";

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  J1R.attach(32, 544, 2500);
  J1L.attach(25, 544, 2500);
  J1R.setTimerWidth(bitRes);
  J1L.setTimerWidth(bitRes);
  J1R.writeTicks(currTickR); 
  J1L.writeTicks(currTickL);

}

// if bitRes = 14, try 4 or 5 ticks
// if bitRes = 12, try 1 or 2 ticks
int ticks = 1;

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

  // Ensure the background operating system stays stable
  yield(); 
}

/**
 * Parses the string buffer and updates the physical hardware registers
 */
void processCommand(String cmd) {
  cmd.trim(); // Clean up accidental spaces
  if (cmd.length() < 2) return; // Ignore empty/broken inputs

  // Extract the motor target identifier (First character)
  char motorSelect = toupper(cmd.charAt(0));
  
  // Extract the numerical microsecond value (Rest of the string)
  String numPart = cmd.substring(1);
  int microSeconds = numPart.toInt();

  // Safety Constraint: Protect the physical internal gear stops
  if (microSeconds < 544 || microSeconds > 2500) {
    Serial.print("ERROR: "); Serial.print(microSeconds);
    Serial.println(" µs is outside safe physical servo limits (544-2400)!");
    return;
  }

  // Calculate exact, hardware-aligned ticks
  int targetTick = round(microSeconds / tau);

  // Execute the hardware register write based on user input
  if (motorSelect == 'L') {
    J1L.writeTicks(targetTick);
    Serial.print("SUCCESS: Left Motor (J1L) set to ");
  } 
  else if (motorSelect == 'R') {
    J1R.writeTicks(targetTick);
    Serial.print("SUCCESS: Right Motor (J1R) set to ");
  } 
  else {
    Serial.print("ERROR: Unknown motor target '"); Serial.print(motorSelect);
    Serial.println("'. Use 'L' or 'R'.");
    return;
  }

  // Print diagnostics back to user for verification
  Serial.print(microSeconds); Serial.print(" µs (Hardware Ticks: ");
  Serial.print(targetTick); Serial.println(")");
}

