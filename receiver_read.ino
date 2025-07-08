#include <Wire.h>
#include "ppm.h"
#include <Servo.h>

Servo ESC1; // Front Left  (CW)
Servo ESC2; // Front Right (CCW)
Servo ESC3; // Rear Right  (CW)
Servo ESC4; // Rear Left   (CCW)

#define THROTTLE 3
#define ROLL     1
#define PITCH    2
#define YAW      4

const long interval = 20; // 50Hz update rate
unsigned long previousMillis = 0;

// MPU6050 variables
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY;
float Acc_angle[2], Gyro_rate[2], Total_angle[2] = {0, 0};
float elapsedTime, time, timePrev;
float rad_to_deg = 180.0 / 3.141592654;

// PID variables
double kp = 1, ki = 0, kd = 0;
float error_pitch, error_roll, prev_error_pitch = 0, prev_error_roll = 0;
float pid_i_pitch = 0, pid_i_roll = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Initialize MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  ppm.begin(A0, false);

  ESC1.attach(9);
  ESC2.attach(5);
  ESC3.attach(3);
  ESC4.attach(6);

  // Start ESCs at minimum throttle (1000us)
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  ESC3.writeMicroseconds(1000);
  ESC4.writeMicroseconds(1000);

  delay(5000); // wait for ESCs to initialize

  time = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < interval) return;
  previousMillis = currentMillis;

  // Update timing
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000.0;

  // Read PPM channels
  long rcThrottle = ppm.read_channel(THROTTLE);
  long rcRoll     = ppm.read_channel(ROLL);
  long rcPitch    = ppm.read_channel(PITCH);
  long rcYaw      = ppm.read_channel(YAW);

  // Read MPU6050 accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  Acc_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
  Acc_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;

  // Read MPU6050 gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  Gyro_rate[0] = Gyr_rawX / 131.0;
  Gyro_rate[1] = Gyr_rawY / 131.0;

  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_rate[0] * elapsedTime) + 0.02 * Acc_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_rate[1] * elapsedTime) + 0.02 * Acc_angle[1];

  float roll = Total_angle[0];
  float pitch = Total_angle[1];

  // PID for pitch
  error_pitch = pitch - 0; // desired pitch = 0 (level)
  pid_i_pitch += (abs(error_pitch) < 3) ? (ki * error_pitch) : 0;
  float pid_p_pitch = kp * error_pitch;
  float pid_d_pitch = kd * (error_pitch - prev_error_pitch) / elapsedTime;
  float PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  prev_error_pitch = error_pitch;

  // PID for roll
  error_roll = roll - 0; // desired roll = 0 (level)
  pid_i_roll += (abs(error_roll) < 3) ? (ki * error_roll) : 0;
  float pid_p_roll = kp * error_roll;
  float pid_d_roll = kd * (error_roll - prev_error_roll) / elapsedTime;
  float PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  prev_error_roll = error_roll;

  // Use rcThrottle as base throttle, constrain 1000-2000us
  rcThrottle = constrain(rcThrottle, 1000, 2000);

  // Mixer to adjust motors based on PID
  // Note: Adjust signs depending on motor directions and your frame config
  int m1 = rcThrottle - PID_pitch - PID_roll; // Front Left  (CW)
  int m2 = rcThrottle - PID_pitch + PID_roll; // Front Right (CCW)
  int m3 = rcThrottle + PID_pitch + PID_roll; // Rear Right  (CW)
  int m4 = rcThrottle + PID_pitch - PID_roll; // Rear Left   (CCW)

  // Limit PWM signals to ESC range
  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  // Write signals to ESCs
  ESC1.writeMicroseconds(m1);
  ESC2.writeMicroseconds(m2);
  ESC3.writeMicroseconds(m3);
  ESC4.writeMicroseconds(m4);

  // Debug output
  Serial.print("Throttle: "); Serial.print(rcThrottle);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" M1: "); Serial.print(m1);
  Serial.print(" M2: "); Serial.print(m2);
  Serial.print(" M3: "); Serial.print(m3);
  Serial.print(" M4: "); Serial.println(m4);
}
