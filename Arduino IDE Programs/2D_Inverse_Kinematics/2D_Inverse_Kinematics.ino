#include <Servo.h>
#include <math.h>

// Pin Definitions
const int shoulderPin = 5;
const int elbowPin = 3;

// Link Lengths in mm
const float L1 = 155.0; 
const float L2 = 155.0;

// Servo MIN and MAX range
const int shoulderMinRange = 0;
const int shoulderMaxRange = 145;
const int elbowMinRange = 0;
const int elbowMaxRange = 180;


Servo shoulderServo;
Servo elbowServo;

/* The Logic Flow
Calculate the distance (r) to the target
Check for "Reachability": If r > L1 + L2, the target is too far
Calculate theta2 (Elbow): Using the Law of Cosines (but atan2 would be better to cover signs better)
Calculate theta1 (Shoulder): Combining the target angle and the internal triangle angle
Map to Servo units: Convert radians to degrees.
*/

const int SERVO_MIN_PULSE = 500;  // Microseconds for 0 degrees
const int SERVO_MAX_PULSE = 2500; // Microseconds for 270 degrees
const int SERVO_RANGE = 270;      // Total range in degrees

// Helper function to protect arm's motion with limits and constraints
void armGuard(float r, float y) {
  // 1. Max Reach Check
  if (r > (L1 + L2) || r < abs(L1 - L2)) {
    Serial.println("Target out of reach!");
    return;
  }

  // 2. Self-Collision Check (The "Fold" Limit)
  if (r < 40) { // Don't allow targets closer than 40mm
    Serial.println("Too close! Potential collision.");
    return;
  }

  // 3. Floor/Table Check
  if (y < 0) {
    Serial.println("Cannot reach below the base.");
    return;
  }
}


void moveArm(float x, float y, bool elbowUp) {
  float r = sqrt(x*x + y*y);
  //armGuard(r, y);

  // 1. SAFETY CHECK: The "Floor" (y-axis)
  // Ensure the arm stays at least 10mm above the base plane
  //float safeY = y;
  
  // 2. SAFETY CHECK: The "Reach" (Radius)
  //float safeX = x;
  
  // If too far out, pull it back to the 300mm limit
  if (r > 300.0) {
    Serial.println("Warning: Target out of reach. Clipping to boundary.");
    Serial.print("Invalid (x, y) coordinates: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    float scale = 300.0 / r;
    x = x * scale;
    y = y * scale;

    Serial.print("Changed to: ");
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");
  }
  
  // If too close in, push it out to the 50mm limit
  if (r < 50.0 && r > 0) {
    Serial.println("Warning: Target too close. Pushing to safe minimum.");
    Serial.print("Invalid (x, y) coordinates: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    float scale = 50.0 / r;
    x = x * scale;
    y = y * scale;

    Serial.print("Changed to: ");
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");
  }

  if (r == 0){
    Serial.print("Invalid (x, y) coordinates: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");
    return;
  }


  float cosTheta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
  float sinTheta2 = sqrt(1 - pow(cosTheta2, 2)); 
  
  // Toggle solution
  if (elbowUp) sinTheta2 = -sinTheta2; 

  float theta2 = atan2(sinTheta2, cosTheta2);
  float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

  float shoulderDeg = degrees(theta1);
  float elbowDeg = degrees(theta2);

  // --- ADJUSTED MAPPING ---
  
  // 1. Shoulder: Add 180 to flip it from Negative X to Positive X
  float finalShoulder = shoulderDeg; 
  
  // 2. Elbow: Often requires an offset so 0 rad (straight) = a specific servo angle
  // Let's assume 135 is "straight" for your elbow
  float finalElbow = 135.0 + elbowDeg; 

  // 3. Convert to microseconds
  int shoulderuS = mapFloat(finalShoulder, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int elbowuS = mapFloat(finalElbow, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  shoulderServo.writeMicroseconds(constrain(shoulderuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  elbowServo.writeMicroseconds(constrain(elbowuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));

  Serial.print("End effector currently sitting at: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(")");

  Serial.print("Shoulder Theta1: ");
  Serial.println(shoulderDeg);
  Serial.print("Elbow Theta2: ");
  Serial.println(elbowDeg);
}

// Helper function because the built in map() only uses integers
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
  Serial.begin(9600);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  // Move to a safe starting position
  shoulderServo.write(90);
  elbowServo.write(90);
  delay(1000);
}

void loop() {
  // Test Cases (x,y)

  // (0, L1 + L2) Full Extension Up: straight up y axis
  // moveArm(0, L1 + L2, true);
  // delay(2000);

  // // (L1 + L2, 0) Full Extension Out: perfectly flat, horizontal x axis
  // moveArm(L1 + L2, 0, true);
  // delay(2000);

  // // (L1, L2) Right Angle: shoulder at 45 degrees and elbow at 90 degrees
  // moveArm(L1, L2, true);
  // delay(2000);

  // 1. Full Extension Up: Straight up the Y-axis (Total height = 300)
  // Shoulder should be 90°, Elbow should be 0°
  // moveArm(0, 310, true); 
  // delay(2000);

  // // 2. Full Extension Out: Perfectly flat along the X-axis (Total length = 300)
  // // Shoulder should be 0°, Elbow should be 0°
  // moveArm(310, 0, true); 
  // delay(2000);

  // // 3. Right Angle (L-Shape): Shoulder vertical, Elbow horizontal
  // // This places the tip at X = 150 (length of L2) and Y = 150 (height of L1)
  // // Shoulder should be 90°, Elbow should be 90°
  // moveArm(155, 155, true); 
  // delay(2000);

  // // 4. The "Inverse" L-Shape: Shoulder horizontal, Elbow vertical
  // // Shoulder should be 0°, Elbow should be 90°
  // moveArm(155, 155, false); // Switching to elbow-down for this pose
  // delay(2000);

  // // 5. The "Perfect" Diagonal: Shoulder and elbow at 45 degrees to ground
  // // Shoulder should be 45, Elbow should be 0
  // moveArm(219.2031, 219.2031, true);
  // delay(2000);

  // moveArm(30, 100, true);
  // delay(3000);
  // moveArm(30, 100, false);
  // delay(3000);
  // moveArm(100, 30, true);
  // delay(3000);
  // moveArm(100, 30, false);
  // delay(3000);
  // moveArm(50, 80, true);
  // delay(3000);
  // moveArm(50, 80, false);
  // delay(3000);
  // moveArm(80, 50, true);
  // delay(3000);
  // moveArm(80, 50, false);
  // delay(3000);
  // moveArm(65, 65, true);
  // delay(3000);
  // moveArm(65, 65, false);
  // delay(3000);

  moveArm(10, 10, true);
  delay(2000);

}
