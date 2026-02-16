#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Pixy2.h>

//////////////////////////////
// ===== ROBOT GEOMETRY =====
//////////////////////////////

#define L1 13.5
#define L2 14.7

//////////////////////////////
// ===== SERVOS ============
//////////////////////////////

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

#define BASE_PIN 2
#define SHOULDER_PIN 3
#define ELBOW_PIN 5

#define BASE_MIN 10
#define BASE_MAX 170
#define SHOULDER_MIN 15
#define SHOULDER_MAX 165
#define ELBOW_MIN 0
#define ELBOW_MAX 180

#define BASE_OFFSET 90
#define ELBOW_OFFSET 0

float baseAngle = 90;
float shoulderAngle = 90;
float elbowAngle = 90;

//////////////////////////////
// ===== TOF CONFIG ========
//////////////////////////////

#define XSHUT1 6
#define XSHUT2 7

Adafruit_VL53L0X tofX = Adafruit_VL53L0X();
Adafruit_VL53L0X tofY = Adafruit_VL53L0X();

//////////////////////////////
// ===== PIXY CONFIG =======
//////////////////////////////

Pixy2 pixy;

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

void moveSmooth(float b1, float s1, float e1, int steps=50){

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

    delay(20);
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
  b = degrees(atan2(y,x)) + BASE_OFFSET;

  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + z*z);

  if (d > (L1+L2) || d < abs(L1-L2))
    return false;

  float phi = atan2(z,r);

  float cos_beta = (L1*L1 + d*d - L2*L2)/(2*L1*d);
  float beta = acos(constrain(cos_beta,-1,1));

  s = degrees(phi + beta);

  float cos_gamma = (L1*L1 + L2*L2 - d*d)/(2*L1*L2);
  e = degrees(acos(constrain(cos_gamma,-1,1))) + ELBOW_OFFSET;


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

void moveToXYZ(float x, float y, float z){

  float b,s,e;

  if(!inverseKinematics(x,y,z,b,s,e)){
    Serial.println("Out of reach");
    return;
  }

  b += getVisualCorrection();
  s += getSurfaceCorrection();

  moveSmooth(b,s,e,60);
  Serial.print("Base: ");
  Serial.println(b);
  Serial.print("Shoulder: ");
  Serial.println(s);
  Serial.print("Elbow: ");
  Serial.println(e);

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

  initTOF();
  pixy.init();

  Serial.println("Precision Paint Arm Ready");
}

//////////////////////////////
// ===== LOOP ==============
//////////////////////////////

void loop(){

  // moveToXYZ(1, 1, 1);
  // delay(2000);
  // moveToXYZ(2, 2, 2);
  // delay(2000);
  // moveToXYZ(3, 3, 3);
  // delay(2000);
  // moveToXYZ(4, 4, 4);
  // delay(2000);
  // moveToXYZ(5, 5, 5);
  // delay(2000);
  // moveToXYZ(6, 6, 6);
  // delay(2000);
  // moveToXYZ(7, 7, 7);
  delay(2000);
  moveToXYZ(8, 8, 8);
  delay(2000);
  moveToXYZ(9, 9, 9);
  delay(2000);
  moveToXYZ(10, 10, 10);
  delay(2000);
  moveToXYZ(15, 15, 15);

  // moveToXYZ(12,0,8);
  // delay(2000);

  // moveToXYZ(15,3,7);
  // delay(2000);

  // moveToXYZ(L1 + L2 - 10, 0, 0);

  // moveToXYZ(20,0,0);
  // delay(2000);

  // elbowServo.write(90);
  // delay(2000);
  // elbowServo.write(120);
  // delay(2000);
  // elbowServo.write(60);
  // delay(2000);

  // baseServo.write(90);
  // delay(2000);
  // baseServo.write(60);
  // delay(2000);
  // baseServo.write(120);
  // delay(2000);

}
