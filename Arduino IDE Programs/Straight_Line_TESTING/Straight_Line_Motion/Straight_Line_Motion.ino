#include <Servo.h>
#include <math.h>

// --------------------
// Arm geometry
// --------------------
const float L1 = 28.5;   // cm, first link
const float L2 = 14;    // cm, second link

// Servo objects
Servo shoulderServo;
Servo elbowServo;

// Servo pins
const int SHOULDER_PIN = 4;
const int ELBOW_PIN = 3;

// Calibration offsets (adjust after testing)
float shoulder_offset = 0;   // degrees
float elbow_offset = 0;      // degrees

const bool SHOULDER_INVERTED = false;
const bool ELBOW_INVERTED    = true;

int servoAngle(float angle_deg, float offset, bool inverted) {
  float a = angle_deg + offset;
  if (inverted) {
    a = 180.0 - a;
  }
  return constrain(a, 0, 180);
}


// --------------------
// Kinematics Functions
// ---------
// -----------

// Compute inverse kinematics for given (x, y)
bool inverseKinematics(float x, float y, float &theta1_deg, float &theta2_deg) {
  float r2 = x*x + y*y;
  float c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2);

  // Check reachability
  if (c2 < -1.0 || c2 > 1.0) return false;

  float s2 = sqrt(1 - c2*c2);  // elbow-up configuration

  float theta2 = atan2(s2, c2);
  float theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));

  theta1_deg = theta1 * 180.0 / PI;
  theta2_deg = theta2 * 180.0 / PI;

  return true;

}


void setup() {
  Serial.begin(9600);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  shoulderServo.write(100);
  delay(5000);
  elbowServo.write(155);
  delay(5000);

  Serial.println("Starting line movement demo...");
}

// --------------------
// Move in straight line
// --------------------
void moveInLine(float x0, float y0, float x1, float y1, int steps, int delay_ms) {
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float x = x0 + t * (x1 - x0);
    float y = y0 + t * (y1 - y0);

    float theta1, theta2;
    if (inverseKinematics(x, y, theta1, theta2)) {
      // Apply offsets and write to servos
    shoulderServo.write(servoAngle(theta1, shoulder_offset, SHOULDER_INVERTED));
    elbowServo.write(servoAngle(theta2, elbow_offset, ELBOW_INVERTED));

      Serial.print("x: "); Serial.print(x);
      Serial.print(", y: "); Serial.print(y);
      Serial.print(", θ1: "); Serial.print(theta1);
      Serial.print(", θ2: "); Serial.println(theta2);
    } else {
      Serial.println("Target out of reach!");
    }

    delay(delay_ms);
  }
}


void loop() {
  // Example: move horizontally
  moveInLine(12, 8, 16, 8, 30, 100);
  delay(2000);

  // Example: move vertically
  moveInLine(16, 8, 16, 12, 30, 100);
  delay(2000);

  // Move back to start
  moveInLine(16, 12, 12, 8, 30, 100);
  delay(2000);
}
