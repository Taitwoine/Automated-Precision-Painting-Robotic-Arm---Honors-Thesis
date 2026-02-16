#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X sensorX = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorY = Adafruit_VL53L0X();

#define X 2
#define Y 3

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(X, OUTPUT);
  pinMode(Y, OUTPUT);

  // Turn both off
  digitalWrite(X, LOW);
  digitalWrite(Y, LOW);
  delay(100);

  // Turn on sensor 1 (X-axis)
  digitalWrite(X, HIGH);
  delay(100);

  if (!sensorX.begin(0x30)) {
    Serial.println("Sensor X failed");
    while (1);
  }  // new I2C address

  // Turn on sensor 2 (Y-axis)
  digitalWrite(Y, HIGH);
  delay(100);

  if (!sensorY.begin(0x31)) {
    Serial.println("Sensor Y failed");
    while (1);
  }  // different address

  Serial.println("Both sensors initialized!");



}

void loop() {
  VL53L0X_RangingMeasurementData_t measureX;
  VL53L0X_RangingMeasurementData_t measureY;

  sensorX.rangingTest(&measureX, false);
  sensorY.rangingTest(&measureY, false);

  Serial.print("X-axis: "); Serial.print(measureX.RangeMilliMeter);
  Serial.print(" mm | Y-axis: "); Serial.print(measureY.RangeMilliMeter);
  Serial.println(" mm");

  delay(500);
}
