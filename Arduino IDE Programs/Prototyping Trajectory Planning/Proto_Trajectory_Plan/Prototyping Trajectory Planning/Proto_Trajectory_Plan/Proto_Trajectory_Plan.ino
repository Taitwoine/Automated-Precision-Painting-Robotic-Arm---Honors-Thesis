#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Pixy2.h>

//////////////////////////////
// ===== ROBOT GEOMETRY =====
//////////////////////////////

#define L1 9.0
#define L2 13.5

//////////////////////////////
// ===== SERVOS ============
//////////////////////////////

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

#define BASE_PIN 2
#define SHOULDER_PIN 3
#define ELBOW_PIN 5

#define BASE_MIN 0
#define BASE_MAX 270
#define SHOULDER_MIN 45
#define SHOULDER_MAX 225
#define ELBOW_MIN 45
#define ELBOW_MAX 225

// #define BASE_OFFSET 90
// #define ELBOW_OFFSET -56.5
// #define SHOULDER_OFFSET 0

float baseAngle = 135;
float shoulderAngle = 135;
float elbowAngle = 135;

//////////////////////////////
// ===== TOF CONFIG ========
//////////////////////////////

#define XSHUT1 6
#define XSHUT2 7

Adafruit_VL53L0X tofX = Adafruit_VL53L0X();
Adafruit_VL53L0X tofY = Adafruit_VL53L0X();

//////////////////////////////
// ======= PIXY CONFIG =======
//////////////////////////////

Pixy2 pixy;


//////////////////////////////
// ===== LOW PASS FILTER =====
//////////////////////////////

// static float filteredVisual = 0;
// static float filteredSurface = 0;

// filteredVisual = 0.8 * filteredVisual + 0.2 * getVisualCorrection();
// filteredSurface = 0.8 * filteredSurface + 0.2 * getSurfaceCorrection();

// b += filteredVisual;
// s += filteredSurface;

//////////////////////////////
// == MILLIS() BASED TIMING ==
//////////////////////////////

// if(millis() - lastUpdate >= 15){
//     // update servos
//     lastUpdate = millis();
// }

//////////////////////////////
// ===== SERVO CONTROL =====
//////////////////////////////

void setServoSafe(Servo &servo, float angle, float minA, float maxA){
  angle = constrain(angle, minA, maxA);
  servo.write(angle);
}

//////////////////////////////
// ===== SMOOTH TRAJECTORY ==
//////////////////////////////

float smoothStep(float t){
  return 3*t*t - 2*t*t*t;
}

void moveSmooth(float b1, float s1, float e1, int steps=10){

  float b0 = baseAngle;
  float s0 = shoulderAngle;
  float e0 = elbowAngle;

  for(int i=0;i<=steps;i++){

    float t = (float)i/steps;
    float k = smoothStep(t);

    float b = b0 + (b1-b0)*k;
    float s = s0 + (s1-s0)*k;
    float e = e0 + (e1-e0)*k;

    setServoSafe(baseServo, b, BASE_MIN, BASE_MAX);
    setServoSafe(shoulderServo, s, SHOULDER_MIN, SHOULDER_MAX);
    setServoSafe(elbowServo, e, ELBOW_MIN, ELBOW_MAX);

    //delay(15);
  }

  baseAngle=b1;
  shoulderAngle=s1;
  elbowAngle=e1;
}

//////////////////////////////
// ===== INVERSE KINEMATICS =
//////////////////////////////

bool inverseKinematics(float x, float y, float z,
                       float &b, float &s, float &e)
{
   b = degrees(atan2(y,x));
  // b = degrees(atan2(y,x)) + BASE_OFFSET;

  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + z*z);

  if (d > (L1+L2) || d < abs(L1-L2))
    return false;

  float phi = atan2(z,r);

  float cos_beta = (L1*L1 + d*d - L2*L2)/(2*L1*d);
  float beta = acos(constrain(cos_beta,-1,1));
  // float beta = acos(constrain(cos_beta,-1,1)) - SHOULDER_OFFSET;

  s = degrees(phi + beta);

  float cos_gamma = (L1*L1 + L2*L2 - d*d)/(2*L1*L2);
  e = degrees(acos(constrain(cos_gamma,-1,1)));
  // e = degrees(acos(constrain(cos_gamma,-1,1))) + ELBOW_OFFSET;


  return true;
}

//////////////////////////////
// ===== TOF INITIALIZATION =
//////////////////////////////

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

//////////////////////////////
// ===== SENSOR CORRECTIONS =
//////////////////////////////

float getSurfaceCorrection(){

  VL53L0X_RangingMeasurementData_t m1, m2;

  tofX.rangingTest(&m1, false);
  tofY.rangingTest(&m2, false);

  if(m1.RangeStatus!=4 && m2.RangeStatus!=4){

    float avg = (m1.RangeMilliMeter + m2.RangeMilliMeter)/2.0;
    float desired = 80.0;  // desired mm distance
    float error = desired - avg;

    return error * 0.02;   // gain
  }

  return 0;
}

float getVisualCorrection(){

  pixy.ccc.getBlocks();

  if(pixy.ccc.numBlocks){
    int error = pixy.ccc.blocks[0].m_x - 160;
    return error * 0.05;
  }

  return 0;
}

//////////////////////////////
// ===== MOVE TO XYZ =======
//////////////////////////////

//X -> Shoulder + elbow, arm reaches forward
//Y -> Base rotation, arm swings left/right
//Z -> Shoulder lift, arm raises up

void moveToXYZ(float x, float y, float z){

  float b,s,e;

  if(!inverseKinematics(x,y,z,b,s,e)){
    Serial.println("Out of reach");
    return;
  }

  b += getVisualCorrection();
  s += getSurfaceCorrection();

  //moveSmooth(b,s,e,60);

  if(abs(b - baseAngle) > 0.5)
    baseServo.write(b);

  if(abs(s - shoulderAngle) > 0.5)
    shoulderServo.write(s);

  if(abs(e - elbowAngle) > 0.5)
    elbowServo.write(e);

  baseAngle = b;
  shoulderAngle = s;
  elbowAngle = e;

  delay(15);

  Serial.print("x value: ");
  Serial.println(x);
  Serial.print("Y value: ");
  Serial.println(y);
  Serial.print("Z value: ");
  Serial.println(z);
  delay(1000);


  Serial.print("Base: ");
  Serial.println(b);
  Serial.print("Shoulder: ");
  Serial.println(s);
  Serial.print("Elbow: ");
  Serial.println(e);

}


void moveLineXYZ(float x0, float y0, float z0,
                 float x1, float y1, float z1,
                 int steps = 10)
{
  for(int i = 0; i <= steps; i++){

    float t = (float)i / steps;

    // Linear interpolation in Cartesian space
    float x = x0 + (x1 - x0) * t;
    float y = y0 + (y1 - y0) * t;
    float z = z0 + (z1 - z0) * t;

    float b,s,e;

    if(inverseKinematics(x,y,z,b,s,e)){
      setServoSafe(baseServo, b, BASE_MIN, BASE_MAX);
      setServoSafe(shoulderServo, s, SHOULDER_MIN, SHOULDER_MAX);
      setServoSafe(elbowServo, e, ELBOW_MIN, ELBOW_MAX);

      //delay(20);
    }
  }

  baseAngle = baseServo.read();
  shoulderAngle = shoulderServo.read();
  elbowAngle = elbowServo.read();
}

void moveLinearXYZ(float x0, float y0, float z0,
                   float x1, float y1, float z1,
                   int steps = 10)
{
  for(int i = 0; i <= steps; i++){

    float t = (float)i / steps;

    float x = x0 + (x1 - x0) * t;
    float y = y0 + (y1 - y0) * t;
    float z = z0 + (z1 - z0) * t;

    moveToXYZ(x, y, z);
  }
}

//////////////////////////////
// ===== SETUP =============
//////////////////////////////

void setup(){

  Serial.begin(115200);
  SPI.begin();
  Wire.begin();

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  baseServo.write(135);
  shoulderServo.write(135);
  elbowServo.write(135);

  initTOF();
  pixy.init();

  Serial.println("Precision Paint Arm Ready");

  
}

//////////////////////////////
// ===== LOOP ==============
//////////////////////////////

void loop(){



  // Paint Down to Up
  // Serial.println("Beginning paint down to up");
  // moveLinearXYZ(7, 0, 8,
  //               21, 0, 8,
  //               80);
  // delay(500);
  // Serial.println("Beginning move over a bit");
  // moveLinearXYZ(21, 0, 8,
  //               21, 1, 8,
  //               20);
  // delay(500);
  // // Paint up to down
  // Serial.println("Beginning paint up to down");
  // moveLinearXYZ(21, 0, 9,
  //               7, 0, 9,
  //               80);

  // delay(2000);
  // Paint left to right
  // moveLinearXYZ(14, -8, 8,
  //             14,  8, 8,
  //             80);
  // //delay(100);
  // // Move down slightly
  // moveLinearXYZ(14, 8, 8,
  //             14, 8, 9,
  //             20);
  // //delay(100);
  // // Paint right to left
  // moveLinearXYZ(14, 8, 9,
  //             14,  -8, 9,
  //             80);
  // //delay(100);
  // moveLinearXYZ(14, -8, 9,
  //             14, -8, 10,
  //             20);
  // //delay(100);
  // moveLinearXYZ(14, -8, 10,
  //             14,  8, 10,
  //             80);
  // //delay(100);
  // moveLinearXYZ(14, 8, 10,
  //             14, 8, 11,
  //             20);
  // //delay(100);
  // moveLinearXYZ(14, 8, 11,
  //             14,  -8, 11,
  //             80);
  // //delay(100);
  // moveLinearXYZ(14, -8, 11,
  //             14, -8, 12,
  //             20);
  // //delay(100);
  // moveLinearXYZ(14, -8, 12,
  //             14,  8, 12,
  //             80);
  // delay(100);


 // moveToXYZ(0, 0, 0);
  moveToXYZ(5, 12, 5);




  // BASE DEBUG
  // moveToXYZ(14, -8, 8);
  // delay(1000);
  // moveToXYZ(14, 8, 8);
  // delay(1000);

  // moveLineXYZ(10, 0, 8,   18, 0, 8);
  // moveLineXYZ(14, -8, 8,   14, 8, 8);

  // moveToXYZ(7, 0, 8);
  //  delay(1000);
  // moveToXYZ(21, 0, 8);
  //  delay(1000);
  // moveToXYZ(7, 0, 8);
  // delay(1000);
  // moveToXYZ(21, 0, 8);
  //  delay(1000);

  // delay(2000);

  // moveToXYZ(8, 0, 7);
  // moveToXYZ(8, 0, 21);
  // moveToXYZ(8, 0, 7);
  // moveToXYZ(8, 0, 21);


//Straight line along X
  // for(float x = 10; x <= 18; x += 0.5){
  //   moveToXYZ(x, 0, 8);
  // }

// Straight line along Y
  // for(float y=-8; y<=8; y+=0.5){
  //   moveToXYZ(14, y, 8);
  // }
  


}


// Geometric Kinematics
// Arm servo attachment stuff and end effector
// Inverse kinematics


// Potentially low on mA
// What's the longest distance to move
// What's the longest time to move
// Minimum time per step