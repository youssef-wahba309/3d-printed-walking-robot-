#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

// === Servo Drivers ===
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);

// === IMU ===
Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;
Kalman kalmanX;

// === Servo Config ===
#define SERVO_MIN  120
#define SERVO_MAX  600

int servo_position[18];
int neutral_position[18] = {90,150,150,30,90,90,30,30,30,
                            90,30,30,150,90,90,150,150,150};

// === Kalman Variables ===
unsigned long lastKalmanTime = 0;

// === Walking Sequence (individual arrays) ===
int walk1[18] = {80,150,150,30,90,90,30,30,30,  70,30,30,150,70,90,150,150,150};
int walk2[18] = {80,150,150,30,90,90,30,30,30,  70,30,60,120,70,90,160,150,150};
int walk3[18] = {90,150,150,30,90,90,30,30,30,  90,30,60,120,90,90,150,150,150};
int walk4[18] = {110,150,150,30,110,90,30,30,30, 105,30,30,150,90,90,150,150,150};
int walk5[18] = {110,150,120,65,110,90,20,30,30, 105,30,30,150,90,90,120,150,150};
int walk6[18] = {90,150,120,65,90,90,30,30,30,  90,30,30,150,90,90,150,150,150};
float angle=0.0, rate=0.0;

// === Motion Engine ===
void move_servo(int time, int servo_target[]) {
  if (time <= 10) return;

  float step[18];
  for (int i = 0; i < 18; i++) {
    step[i] = (servo_target[i] - servo_position[i]) / (float)time;
  }

  unsigned long start_time = millis();
  unsigned long final_time = start_time + time;

  while (millis() < final_time) {
    unsigned long elapsed = millis() - start_time;

    for (int i = 0; i < 9; i++) {
      int pos1 = servo_position[i] + step[i] * elapsed;
      int pos2 = servo_position[i+9] + step[i+9] * elapsed;

      int pulse1 = map(pos1, 0, 180, SERVO_MIN, SERVO_MAX);
      int pulse2 = map(pos2, 0, 180, SERVO_MIN, SERVO_MAX);

      pca2.setPWM(i, 0, pulse1);
      pca1.setPWM(i, 0, pulse2);
    }
    delay(10); // ~10 ms update
  }

  for (int i = 0; i < 18; i++) {
    servo_position[i] = servo_target[i];
  }
}

// === Kalman Update ===
bool updateKalman(float &angleOut, float &rateOut) {
  if(!mpu.getEvent(&accel, &gyro, &temp)) return false;

  float ax = accel.acceleration.x;
  float az = accel.acceleration.z;
  float gx = gyro.gyro.x * 180.0 / PI;

  float accPitch = atan2(ax, az) * 180.0 / PI;

  unsigned long now = micros();
  float dt = (now - lastKalmanTime) / 1000000.0;
  lastKalmanTime = now;
  if(dt < 0.001) dt = 0.001;

  angleOut = kalmanX.getAngle(accPitch, gx, dt);
  rateOut  = gx;
  return true;
}

// === Walking Sequence ===
char performWalkSequence(int stepDuration) {
  for (int i=0;i<100;i++){
  move_servo(stepDuration, walk1);
   if(updateKalman(angle, rate)) {
    // If tilt exceeds ±50°, return to neutral
    if(abs(angle) > 50.0) {
    move_servo(300, neutral_position);
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);
      return 0;
      }
   }
  move_servo(stepDuration, walk2);
  if(updateKalman(angle, rate)) {
    // If tilt exceeds ±15°, return to neutral
    if(abs(angle) > 50.0) {
      move_servo(300, neutral_position);
       Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);return 0;}
   }
  move_servo(stepDuration, walk3);
  if(updateKalman(angle, rate)) {
    // If tilt exceeds ±15°, return to neutral
    if(abs(angle) > 50.0) {
      move_servo(300, neutral_position);
       Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);return 0;}
   }
  move_servo(stepDuration, walk4);
if(updateKalman(angle, rate)) {
    // If tilt exceeds ±50°, return to neutral
    if(abs(angle) > 50.0) {
      move_servo(300, neutral_position);
       Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);return 0;}
   }
  move_servo(stepDuration, walk5);
  if(updateKalman(angle, rate)) {
    // If tilt exceeds ±15°, return to neutral
    if(abs(angle) > 50.0) {
      move_servo(300, neutral_position);
       Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);return 0;}
   }
  move_servo(stepDuration, walk6);
  if(updateKalman(angle, rate)) {
    // If tilt exceeds ±15°, return to neutral
    if(abs(angle) > 50.0) {
      move_servo(300, neutral_position);
       Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" | Rate: "); Serial.println(rate);
      delay(500);return 0;}
   }
   }
}

// === IMU Calibration ===
void calibrateIMU() {
  Serial.println("Calibrating IMU...");
  float sumPitch = 0;
  int samples = 200;

  for(int i=0; i<samples; i++) {
    if(mpu.getEvent(&accel, &gyro, &temp)) {
      float ax = accel.acceleration.x;
      float az = accel.acceleration.z;
      float accPitch = atan2(ax, az) * 180.0 / PI;
      sumPitch += accPitch;
    }
    delay(10);
  }

  float avgPitch = sumPitch / samples;
  kalmanX.setAngle(avgPitch);  // baseline = 0
  Serial.print("IMU baseline set to: ");
  Serial.println(avgPitch);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  pca1.begin(); pca2.begin();
  pca1.setPWMFreq(50); pca2.setPWMFreq(50);

  if(!mpu.begin()) {
    Serial.println("IMU not found!");
    while(1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for(int i=0; i<18; i++) servo_position[i] = neutral_position[i];
  move_servo(2000, neutral_position);

  calibrateIMU();
  lastKalmanTime = micros();
  Serial.println("System Ready.");
}

// === Loop ===
void loop() {
performWalkSequence(500);
 // 50 Hz loop
}
