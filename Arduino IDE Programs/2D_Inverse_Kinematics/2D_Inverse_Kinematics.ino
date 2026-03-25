#include <Servo.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Wire.h>


// Pin Definitions
const int shoulderPin = 2;
const int elbowPin = 3;
const int wristPin = 5;

// Link Lengths in mm
const float L1 = 155.0; 
const float L2 = 155.0; // May need to debug L2 + L3 for new length to end effector (hand) position
const float L3 = 23.0;

// Current X and Y link lengths
float currentX = 155.0 + L3; 
float currentY = 155.0;

// Servo Initial States
const int initialShoulder = 60;
const int initialElbow = 45;
const int initialWrist = 125;


Servo shoulderServo;
Servo elbowServo;
Servo wristServo;

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

// TOF CONFIGURATION
#define XSHUT1 6
#define XSHUT2 7

Adafruit_VL53L0X tofX = Adafruit_VL53L0X();
Adafruit_VL53L0X tofY = Adafruit_VL53L0X();

//PIXY CONFIGURATION
Pixy2 pixy;

// TOF INITIALIZATION
void initTOF(){
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(100);

  digitalWrite(XSHUT1, HIGH);
  delay(100);
  tofX.begin(0x30);

  digitalWrite(XSHUT2, HIGH);
  delay(100);
  tofY.begin(0x31);
}
//-----------------------------------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------------------------------
// Helper function because the built in map() only uses integers
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//-----------------------------------------------------------------------------------------------------
void moveArm(float x, float y, bool elbowUp) {
  float r = sqrt(x*x + y*y);
  //armGuard(r, y);
  
  // If too far out, pull it back to the 310mm limit
  if (r > 310.0) {
    Serial.println("Warning: Target out of reach. Clipping to boundary.");
    Serial.print("Invalid (x, y) coordinates: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");

    float scale = 310.0 / r;
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
  float wristDeg = -(shoulderDeg + elbowDeg);

  // --- ADJUSTED MAPPING ---
  
  // 1. Shoulder: Add 180 to flip it from Negative X to Positive X
  float finalShoulder = shoulderDeg; 
  
  // 2. Elbow: Often requires an offset so 0 rad (straight) = a specific servo angle
  // Let's assume 135 is "straight" for your elbow
  float finalElbow = 135.0 + elbowDeg; 

  // 3. Wrist: Maintain parallel to floor/table
  float finalWrist = 204 + wristDeg;

  // Convert to microseconds
  int shoulderuS = mapFloat(finalShoulder, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int elbowuS = mapFloat(finalElbow, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int wristuS = mapFloat(finalWrist, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  shoulderServo.writeMicroseconds(constrain(shoulderuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  elbowServo.writeMicroseconds(constrain(elbowuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  wristServo.writeMicroseconds(constrain(wristuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));

  // Serial.print("End effector currently sitting at: (");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  // Serial.println(")");

  // Serial.print("Shoulder Theta1: ");
  // Serial.println(shoulderDeg);
  // Serial.print("Elbow Theta2: ");
  // Serial.println(elbowDeg);
  // Serial.print("Wrist Theta: ");
  // Serial.println(wristDeg);
}
//-----------------------------------------------------------------------------------------------------
// Linear Interpolation (LERP) aka coordinated motion
// Breaking up movement into a bunch of small "waypoints" and moving through at a set speed rather than asap
// 1. Store current position
// 2. Divide the distance (start and end into small steps, in this case 2mm per step)
// 3. Time the steps (use delay() to control velocity)
void moveSmoothly(float x, float y, bool elbowUp, float speed) {
  // 1. Calculate how far it needs to travel
  float dx = x - currentX;
  float dy = y - currentY;
  float distance = sqrt(dx*dx + dy*dy);
  
  // 2. Determine how many steps to take (e.g., 1 step per 2mm)
  int steps = distance / 2.0; 
  if (steps < 1) steps = 1; // Ensure at least one step

  for (int i = 1; i <= steps; i++) {
    // 3. Calculate the intermediate "waypoint"
    float intermediateX = currentX + (dx * i / steps);
    float intermediateY = currentY + (dy * i / steps);
    
    // 4. Calculates the IK and the level wrist angle for THIS specific spot
    moveArm(intermediateX, intermediateY, elbowUp);
    
    // 5. Control the speed (lower delay = faster motion)
    delay(speed); 
  }

  // 6. Update current position for the next move
  currentX = x;
  currentY = y;
}
//-----------------------------------------------------------------------------------------------------
void runSquarePath() {
  // 1. Get Diameter from User
  while (Serial.available() == 0) {} // Wait for input
  float diameter = Serial.parseFloat();
  
  // Clean the buffer
  while(Serial.available() > 0) Serial.read();

  // Validate Range
  if (diameter < 5 || diameter > 20) {
    Serial.println("Error: Diameter must be between 5mm and 20mm.");
    return;
  }

  // 2. Calculate Radius and Bounding Box
  float radius = diameter / 2.0;
  float minLimit = 40 + radius;
  float maxLimit = 214 - radius;

  Serial.print("Starting path with Radius: ");
  Serial.println(radius);

  // 3. Generate Raster Path (Snake Pattern)
  // Move in increments of the diameter to ensure coverage
  for (float y = minLimit; y <= maxLimit; y += diameter) {
    
    // If Y is an even multiple of diameter, go Left -> Right
    // If Y is an odd multiple, go Right -> Left
    int row = (int)((y - minLimit) / diameter);
    
    if (row % 2 == 0) {
      // Left to Right
      moveSmoothly(minLimit, y, true, 15);
      moveSmoothly(maxLimit, y, true, 15);
    } else {
      // Right to Left
      moveSmoothly(maxLimit, y, true, 15);
      moveSmoothly(minLimit, y, true, 15);
    }
  }
  
  Serial.println("Path Complete!");
  // Return to a neutral home position
  moveSmoothly(155, 155, true, 15);
}
//-----------------------------------------------------------------------------------------------------
void runPolarArcPath() {
  // 1. Get Diameter and Calculate Radius
  while (Serial.available() == 0) {} 
  float diameter = Serial.parseFloat();
  while(Serial.available() > 0) Serial.read();

  if (diameter < 5 || diameter > 20) {
    Serial.println("Error: Keep diameter between 5-20mm.");
    return;
  }

  // 2. Define the Sweep Constraints
  // Angle in degrees: 0 is flat right, 90 is straight up.
  float minAngle = 10.0;  // Start 10 degrees off the X-axis
  float maxAngle = 80.0;  // End 10 degrees before the Y-axis
  
  float maxReach = 310.0 - (diameter / 2.0); // Full extension minus spray radius
  float minReach = 60.0;                     // Minimum extension to avoid base

  Serial.println("Starting Polar Sweep Path...");

  // 3. The Radial Loop (Moving from far reach back toward the base)
  for (float r = maxReach; r >= minReach; r -= diameter) {
    
    // Determine if we are sweeping "Up" or "Down" the arc
    int layer = (int)((maxReach - r) / diameter);
    
    if (layer % 2 == 0) {
      // Sweep from 10 deg to 80 deg
      for (float deg = minAngle; deg <= maxAngle; deg += 2.0) {
        float rad = radians(deg);
        float targetX = r * cos(rad);
        float targetY = r * sin(rad);
        moveSmoothly(targetX, targetY, true, 15); 
        delay(10); // Control sweep speed
      }
    } else {
      // Sweep from 80 deg back to 10 deg
      for (float deg = maxAngle; deg >= minAngle; deg -= 2.0) {
        float rad = radians(deg);
        float targetX = r * cos(rad);
        float targetY = r * sin(rad);
        moveSmoothly(targetX, targetY, true, 15);
        delay(10);
      }
    }
  }

  Serial.println("Polar Path Complete!");
  moveSmoothly(155, 155, true, 15); // Return to home
}
//-----------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  SPI.begin();
  Wire.begin();

  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);

  // Move to a safe starting position
  shoulderServo.write(initialShoulder);
  elbowServo.write(initialElbow);
  wristServo.write(initialWrist);
  delay(1000);

  initTOF();
  pixy.init();
  Serial.println("Ready to begin");
  Serial.println("Enter the diameter of the spray's cone in mm (range is 5-20 mm): ");
  delay(3000);
}
//-----------------------------------------------------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    runSquarePath();
    Serial.println("Enter a new diameter (5-20mm) to run again:");
  }
  if (Serial.available() > 0) {
    runPolarArcPath();
    Serial.println("Enter a new diameter (5-20mm) to run again:");
  }

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

  // moveArm(10, 10, true);
  // delay(2000);

  VL53L0X_RangingMeasurementData_t measureX;
  VL53L0X_RangingMeasurementData_t measureY;

  tofX.rangingTest(&measureX, false);
  tofY.rangingTest(&measureY, false);

  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");
  // delay(500);

  // moveSmoothly(250, 50, true, 20); 
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");
  
  // moveSmoothly(155, 155, true, 20);
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");

  // moveSmoothly(0, 310, true, 20);
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");

  // moveSmoothly(310, 0, true, 20);
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");

  // moveSmoothly(155, 155, false, 20);
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");

  // moveSmoothly(219.20, 219.20, true, 20);
  // delay(1500);
  // tofX.rangingTest(&measureX, false);
  // tofY.rangingTest(&measureY, false);
  // Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  // Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  // Serial.println(" mm");

}



/*
Path planning next step
Path planning algorithms
Or
Figure out how many passes side to side to top to bottom needs to occur
Step 1. Simple path planning, where to paint// Pre program path and surface

step. 2 Compute surface
After, snap shots with camera and coordinating "pictures" to find cornerrs of each and make a larger image

Bounding box coordinates (Locate corners)

1st, assume bounding box coordinates are known

2nd Compute bounding box coordinates and size with camera and sensors

Find radius/diameter of cone spray to compute number of passes*/