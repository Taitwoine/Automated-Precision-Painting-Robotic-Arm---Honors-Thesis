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
const float L1 = 160.0; 
const float L2 = 160.0;
const float L3 = 25.0;

// Current X and Y link lengths
float currentX = 155.0 + L3; 
float currentY = 155.0;

// Servo Initial States
const int initialShoulder = 60;
const int initialElbow = 40;
const int initialWrist = 125;

// Pixel-to-Millimeter Ratios
const int horizontalRatio = 0.44375; // 142 mm / 320 pixels
const int verticalRatio = 0.45416; // 109 mm / 240 pixels
const int averageRatio = 0.449;

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

Adafruit_VL53L0X tofX = Adafruit_VL53L0X();

// TOF INITIALIZATION
void initTOF(){
  pinMode(XSHUT1, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  delay(100);

  digitalWrite(XSHUT1, HIGH);
  delay(100);
  tofX.begin(0x30);
}
//-----------------------------------------------------------------------------------------------------
#define NICLA 0x12 // Default OpenMV Nicla Vision I2C address
// Helper function for communication between Nicla Vision and MKR 1010
void getVisionData() {
  // 1. Request 5 bytes from the Nicla (0x12)
  Wire.requestFrom(0x12, 5); 
  
  if (Wire.available() >= 5) {
    // 2. Read raw bytes and scale back to image dimensions (320x240)
    int x_pix  = Wire.read() * 2; // X coord
    int y_pix  = Wire.read() * 2; // Y coord
    int w_pix  = Wire.read() * 2; // Camera bound width
    int h_pix  = Wire.read() * 2; // Camera bound height
    int z_dist = Wire.read() * 2; // Surface distance from ToF

    // 3. Serial Feedback for Debugging
    Serial.println("--- Vision Update ---");
    Serial.print(" ToF Distance: "); Serial.print(z_dist); Serial.println(" mm");

    // 4. Spatial Calibration (The "Millimeters per Pixel" approach)
    const float mmPerPixel = 0.265; 
    
    // Calculate offsets from the camera center (160, 120)
    float diffX_pixels = x_pix - 160;
    float diffY_pixels = y_pix - 120;

    // Convert pixel offsets to real-world millimeters
    float offsetX_mm = diffX_pixels * mmPerPixel;
    float offsetY_mm = diffY_pixels * mmPerPixel;

    // 5. Map to Robot Coordinates
    // Assuming camera center is physically at Robot (155, 155)
    // Camera Y (up/down) maps to Robot X (reach)
    // Camera X (left/right) maps to Robot Y (side-to-side)
    float targetX = 155.0 - offsetY_mm; // Subtracting makes Camera Top = Max Reach
    float targetY = 155.0 + offsetX_mm; 
    float areaWidth = w_pix * mmPerPixel;

    // 6. Safeguard Clipping
    targetX = constrain(targetX, 50, 310);
    targetY = constrain(targetY, 0, 310);

    Serial.print("Target X: "); Serial.print(targetX);
    Serial.print(" Y: "); Serial.println(targetY);
    Serial.println("Moving to starting point...");

    // 7. Execute the Raster Path
    runSquarePath(targetX, targetY, areaWidth);
  }
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
  float finalElbow = 145.0 + elbowDeg; 

  // 3. Wrist: Maintain parallel to floor/table
  float finalWrist = 195 + wristDeg;

  // Convert to microseconds
  int shoulderuS = mapFloat(finalShoulder, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int elbowuS = mapFloat(finalElbow, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  int wristuS = mapFloat(finalWrist, 0, 270, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  shoulderServo.writeMicroseconds(constrain(shoulderuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  elbowServo.writeMicroseconds(constrain(elbowuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  wristServo.writeMicroseconds(constrain(wristuS, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
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
// Modified to accept parameters
void runSquarePath(float startX, float startY, float totalWidth) {
  // 1. Get Diameter (We still need the spray cone size for overlap logic)
  // Send this from Nicla, keep a default, or ask once in setup
  float diameter = 5.0; // Default 10mm spray cone
  
  // 2. Define limits based on the area the camera found
  float minLimitX = startX;
  float maxLimitX = startX + totalWidth;

  Serial.println("Executing Vision-Guided Raster...");

  // 3. Generate Raster Path (Snake Pattern)
  // We use the width provided by the camera
  for (float y = startY; y >= (startY - totalWidth); y -= diameter) {
    
    int row = (int)(abs(y - startY) / diameter);
    
    if (row % 2 == 0) {
      // Left to Right
      moveSmoothly(minLimitX, y, true, 10);
      delay(100);
      moveSmoothly(maxLimitX, y, true, 10);
    } 
    else {
      // Right to Left
      moveSmoothly(maxLimitX, y, true, 10);
      delay(100);
      moveSmoothly(minLimitX, y, true, 10);
    }
    delay(100);
  }
  
  Serial.println("Vision Path Complete!");
  homePosition();
}
//-----------------------------------------------------------------------------------------------------
// void runPolarArcPath() {
//   // 1. Get Diameter and Calculate Radius
//   while (Serial.available() == 0) {} 
//   float diameter = Serial.parseFloat();
//   while(Serial.available() > 0) Serial.read();

//   if (diameter < 1 || diameter > 20) {
//     Serial.println("Error: Keep diameter between 1-20mm.");
//     return;
//   }

//   // 2. Define the Sweep Constraints
//   // Angle in degrees: 0 is flat right, 90 is straight up.
//   float minAngle = 0.0;  // Start 10 degrees off the X-axis
//   float maxAngle = 100.0;  // End 10 degrees before the Y-axis
  
//   float maxReach = 310.0 - (diameter / 2.0); // Full extension minus spray radius
//   float minReach = 100.0;                     // Minimum extension to avoid base

//   Serial.println("Starting Polar Sweep Path...");

//   // 3. The Radial Loop (Moving from far reach back toward the base)
//   for (float r = maxReach; r >= minReach; r -= diameter) {
    
//     // Determine if we are sweeping "Up" or "Down" the arc
//     int layer = (int)((maxReach - r) / diameter);
    
//     if (layer % 2 == 0) {
//       // Sweep from min to max angle
//       for (float deg = minAngle; deg <= maxAngle; deg += 2.0) {
//         float rad = radians(deg);
//         float targetX = r * cos(rad);
//         float targetY = r * sin(rad);
//         moveSmoothly(targetX, targetY, true, 7); 
//         delay(10); // Control sweep speed
//       }
//     } else {
//       // Sweep from max to min angle
//       for (float deg = maxAngle; deg >= minAngle; deg -= 2.0) {
//         float rad = radians(deg);
//         float targetX = r * cos(rad);
//         float targetY = r * sin(rad);
//         moveSmoothly(targetX, targetY, true, 7);
//         delay(10);
//       }
//     }
//   }

//   Serial.println("Polar Path Complete!");
//   homePosition();
// }

void homePosition(){
  shoulderServo.write(initialShoulder);
  elbowServo.write(initialElbow);
  wristServo.write(initialWrist);
  delay(1000);
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
  homePosition();

  initTOF();
  Serial.println("Ready to begin");
  delay(3000);
}
//-----------------------------------------------------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'v') {
      Serial.println("Requesting coordinates from Nicla...");
      getVisionData();
    }
    
    if (command == 'h') {
      Serial.println("Returning home.");
      homePosition();
    }
  }
}

// 142 mm Width
// 109 mm Height
// 195-200 mm Distance from surface


