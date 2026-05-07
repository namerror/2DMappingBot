#include <SSD1306Wire.h>
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

SSD1306Wire lcd(0x3c, SDA, SCL);
MPU6050 imu;

#define TRIG 5
#define ECHO 16
#define STBY A10
#define PWMA A17
#define PWMB A5
#define AIN1 A13 // A for left motor, B for right motor
#define AIN2 A14
#define BIN1 A15
#define BIN2 A16
#define INT A4

const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_FREQUENCY = 10000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;
const unsigned long DISTANCE_INTERVAL_MS = 100;
const unsigned long IMU_INTERVAL_MS = 20;
const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long DEFAULT_ACTION_DURATION_MS = 250;
const unsigned long MAX_ACTION_DURATION_MS = 10000;
const unsigned long CALIBRATION_DURATION_MS = 5000;
const unsigned long CALIBRATION_SAMPLE_INTERVAL_MS = 20;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;
const float ACCEL_AXIS_STABILITY_THRESHOLD_G = 0.05f;
const float ACCEL_MAG_STABILITY_THRESHOLD_G = 0.06f;

int dutyCycle = 0; // 0-100% duty cycle for motor speed control
unsigned long lastDistanceMillis = 0;
unsigned long lastImuMillis = 0;
unsigned long lastImuSampleMillis = 0;
unsigned long lastControlMillis = 0;
float imuYawDeg = 0.0f;
float gyroZBias = 0.0f;
bool imuConnected = false;
bool imuCalibrated = false;
bool calibrationRunning = false;

enum ActionCommand {
  ACTION_NONE,
  ACTION_FORWARD,
  ACTION_BACKWARD,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_BRAKE
};

ActionCommand activeAction = ACTION_NONE;
unsigned long actionStartMillis = 0;
unsigned long actionDurationMs = 0;
String serialCommandBuffer = "";

float readDistance();
float getAverage();
void setMotorSpeed(int speed);
void moveForward();
void moveBackward();
void stopMotors();
void shortBrake();
void turnLeft();
void turnRight();
void applyActionCommand(ActionCommand command);
void stopActionCommand();
void sendActionCommand(ActionCommand command, unsigned long durationMs);
void updateActionCommand(unsigned long currentMillis);
unsigned long parseDuration(String text);
float maxFloat(float a, float b);
void discardSerialInput();
void sendCalibrationStart();
void sendCalibrationOk(float gzBias, unsigned int sampleCount, float accelAxisRange, float accelMagnitudeRange);
void sendCalibrationFailed(const char* reason, unsigned int sampleCount);
void calibrateImu();
void processSerialCommand(String commandLine);
void readSerialCommands();
void imuLoop();
void distanceSensingLoop();
void controlLoop();

void setup() {
  //Serial.begin(9600);  // ← ADD THIS: start USB communication to laptop
  Serial.begin(115200); // Displays to Serial monitor
  Wire.begin(SDA, SCL);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  lcd.init();
  lcd.flipScreenVertically();
  lcd.setFont(ArialMT_Plain_16);

  // control settings
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH); // Standby off
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW); // Stop motors

  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);
  setMotorSpeed(30);

  imu.initialize();
  imuConnected = imu.testConnection();
  lastImuSampleMillis = millis();
}

float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long timeout = micros() + 26233L;
  while((digitalRead(ECHO)==LOW) && (micros()<timeout));
  unsigned long start_time = micros();
  timeout = start_time + 26233L;
  while((digitalRead(ECHO)==HIGH) && (micros()<timeout));
  unsigned long lapse = micros() - start_time;
  return lapse * 0.01716f;
}

#define K 7
float queue[K];
int qindex = 0;
float lastDistance = 0;
float averageDistance = 0;

float getAverage() {
  float sum = 0;
  for(int i = 0; i < K; i++) {  
    sum += queue[i];
  }
  return sum / K;
}

void setMotorSpeed(int speed) {
  dutyCycle = constrain(speed, 0, 100);
  int pwmValue = map(dutyCycle, 0, 100, 0, MAX_PWM_DUTY);

  ledcWrite(PWM_CHANNEL_A, pwmValue);
  ledcWrite(PWM_CHANNEL_B, pwmValue);
}

void moveForward() {
  // CW for both motors
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void moveBackward() {
  // CCW for both motors
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void shortBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void turnLeft() {
  // Left motor CCW, right motor CW
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void turnRight() {
  // Left motor CW, right motor CCW
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void applyActionCommand(ActionCommand command) {
  switch(command) {
    case ACTION_FORWARD:
      moveForward();
      break;
    case ACTION_BACKWARD:
      moveBackward();
      break;
    case ACTION_LEFT:
      turnLeft();
      break;
    case ACTION_RIGHT:
      turnRight();
      break;
    case ACTION_BRAKE:
      shortBrake();
      break;
    case ACTION_NONE:
    default:
      stopMotors();
      break;
  }
}

void stopActionCommand() {
  activeAction = ACTION_NONE;
  actionDurationMs = 0;
  stopMotors();
}

void sendActionCommand(ActionCommand command, unsigned long durationMs) {
  if(calibrationRunning || !imuCalibrated) {
    stopActionCommand();
    return;
  }

  if(command == ACTION_NONE || durationMs == 0) {
    stopActionCommand();
    return;
  }

  if(durationMs > MAX_ACTION_DURATION_MS) {
    durationMs = MAX_ACTION_DURATION_MS;
  }

  activeAction = command;
  actionStartMillis = millis();
  actionDurationMs = durationMs;
  applyActionCommand(activeAction);
}

void updateActionCommand(unsigned long currentMillis) {
  if(activeAction == ACTION_NONE) {
    return;
  }

  if(currentMillis - actionStartMillis >= actionDurationMs) {
    stopActionCommand();
  }
}

unsigned long parseDuration(String text) {
  text.trim();
  if(text.length() == 0) {
    return DEFAULT_ACTION_DURATION_MS;
  }

  while(text.length() > 0 && !isDigit(text.charAt(0))) {
    text.remove(0, 1);
    text.trim();
  }

  unsigned long durationMs = text.toInt();
  if(durationMs == 0) {
    return DEFAULT_ACTION_DURATION_MS;
  }

  return durationMs;
}

float maxFloat(float a, float b) {
  return a > b ? a : b;
}

void discardSerialInput() {
  while(Serial.available() > 0) {
    Serial.read();
  }
  serialCommandBuffer = "";
}

void sendCalibrationStart() {
  Serial.print(millis());
  Serial.println(",status,calibration,start");
}

void sendCalibrationOk(float gzBias, unsigned int sampleCount, float accelAxisRange, float accelMagnitudeRange) {
  Serial.print(millis());
  Serial.print(",status,calibration,ok,gz_bias,");
  Serial.print(gzBias, 4);
  Serial.print(",samples,");
  Serial.print(sampleCount);
  Serial.print(",accel_axis_range,");
  Serial.print(accelAxisRange, 4);
  Serial.print(",accel_mag_range,");
  Serial.println(accelMagnitudeRange, 4);
}

void sendCalibrationFailed(const char* reason, unsigned int sampleCount) {
  Serial.print(millis());
  Serial.print(",status,calibration,failed,");
  Serial.print(reason);
  Serial.print(",samples,");
  Serial.println(sampleCount);
}

void calibrateImu() {
  stopActionCommand();
  calibrationRunning = true;
  imuCalibrated = false;
  gyroZBias = 0.0f;
  imuYawDeg = 0.0f;
  sendCalibrationStart();

  lcd.clear();
  lcd.drawString(0, 0, "Calibrating");
  lcd.drawString(0, 18, "Keep robot still");
  lcd.display();

  if(!imuConnected) {
    calibrationRunning = false;
    discardSerialInput();
    sendCalibrationFailed("no_imu", 0);
    return;
  }

  bool firstSample = true;
  float minAx = 0.0f;
  float maxAx = 0.0f;
  float minAy = 0.0f;
  float maxAy = 0.0f;
  float minAz = 0.0f;
  float maxAz = 0.0f;
  float minAccelMagnitude = 0.0f;
  float maxAccelMagnitude = 0.0f;
  double gzSum = 0.0;
  unsigned int sampleCount = 0;
  unsigned long startMillis = millis();
  unsigned long lastSampleMillis = 0;

  while(millis() - startMillis < CALIBRATION_DURATION_MS) {
    stopMotors();

    unsigned long currentMillis = millis();
    if(currentMillis - lastSampleMillis >= CALIBRATION_SAMPLE_INTERVAL_MS) {
      lastSampleMillis = currentMillis;

      int16_t rawAx = 0;
      int16_t rawAy = 0;
      int16_t rawAz = 0;
      int16_t rawGx = 0;
      int16_t rawGy = 0;
      int16_t rawGz = 0;
      imu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

      float ax = rawAx / ACCEL_SCALE;
      float ay = rawAy / ACCEL_SCALE;
      float az = rawAz / ACCEL_SCALE;
      float accelMagnitude = sqrt((ax * ax) + (ay * ay) + (az * az));
      float gz = rawGz / GYRO_SCALE;

      if(firstSample) {
        minAx = maxAx = ax;
        minAy = maxAy = ay;
        minAz = maxAz = az;
        minAccelMagnitude = maxAccelMagnitude = accelMagnitude;
        firstSample = false;
      } else {
        if(ax < minAx) minAx = ax;
        if(ax > maxAx) maxAx = ax;
        if(ay < minAy) minAy = ay;
        if(ay > maxAy) maxAy = ay;
        if(az < minAz) minAz = az;
        if(az > maxAz) maxAz = az;
        if(accelMagnitude < minAccelMagnitude) minAccelMagnitude = accelMagnitude;
        if(accelMagnitude > maxAccelMagnitude) maxAccelMagnitude = accelMagnitude;
      }

      gzSum += gz;
      sampleCount++;
    }

    delay(1);
  }

  if(sampleCount == 0) {
    calibrationRunning = false;
    discardSerialInput();
    sendCalibrationFailed("no_samples", sampleCount);
    return;
  }

  float accelAxisRange = maxFloat(maxAx - minAx, maxFloat(maxAy - minAy, maxAz - minAz));
  float accelMagnitudeRange = maxAccelMagnitude - minAccelMagnitude;
  if(accelAxisRange > ACCEL_AXIS_STABILITY_THRESHOLD_G ||
      accelMagnitudeRange > ACCEL_MAG_STABILITY_THRESHOLD_G) {
    calibrationRunning = false;
    discardSerialInput();
    sendCalibrationFailed("unstable", sampleCount);
    return;
  }

  gyroZBias = gzSum / sampleCount;
  imuYawDeg = 0.0f;
  imuCalibrated = true;
  lastImuSampleMillis = millis();
  lastImuMillis = lastImuSampleMillis;
  calibrationRunning = false;
  discardSerialInput();
  sendCalibrationOk(gyroZBias, sampleCount, accelAxisRange, accelMagnitudeRange);

  lcd.clear();
  lcd.drawString(0, 0, "Calibrated");
  lcd.drawString(0, 18, "Yaw reset");
  lcd.display();
}

void processSerialCommand(String commandLine) {
  commandLine.trim();
  if(commandLine.length() == 0) {
    return;
  }

  char command = toupper(commandLine.charAt(0));
  String argument = commandLine.substring(1);

  switch(command) {
    case 'C':
      calibrateImu();
      break;
    case 'F':
      sendActionCommand(ACTION_FORWARD, parseDuration(argument));
      break;
    case 'B':
      sendActionCommand(ACTION_BACKWARD, parseDuration(argument));
      break;
    case 'L':
      sendActionCommand(ACTION_LEFT, parseDuration(argument));
      break;
    case 'R':
      sendActionCommand(ACTION_RIGHT, parseDuration(argument));
      break;
    case 'K':
      sendActionCommand(ACTION_BRAKE, parseDuration(argument));
      break;
    case 'S':
    case 'X':
      stopActionCommand();
      break;
    case 'V':
      setMotorSpeed(argument.toInt());
      break;
    default:
      break;
  }
}

void readSerialCommands() {
  while(Serial.available() > 0) {
    char incoming = Serial.read();

    if(incoming == '\r') {
      continue;
    }

    if(incoming == '\n') {
      processSerialCommand(serialCommandBuffer);
      serialCommandBuffer = "";
      continue;
    }

    if(serialCommandBuffer.length() < 32) {
      serialCommandBuffer += incoming;
    }
  }
}

void imuLoop() {
  if(!imuConnected) {
    return;
  }

  unsigned long currentMillis = millis();
  if(currentMillis - lastImuMillis < IMU_INTERVAL_MS) {
    return;
  }
  lastImuMillis = currentMillis;

  int16_t rawAx = 0;
  int16_t rawAy = 0;
  int16_t rawAz = 0;
  int16_t rawGx = 0;
  int16_t rawGy = 0;
  int16_t rawGz = 0;
  imu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  float ax = rawAx / ACCEL_SCALE;
  float ay = rawAy / ACCEL_SCALE;
  float az = rawAz / ACCEL_SCALE;
  float gx = rawGx / GYRO_SCALE;
  float gy = rawGy / GYRO_SCALE;
  float gz = (rawGz / GYRO_SCALE) - gyroZBias;

  float dtSeconds = (currentMillis - lastImuSampleMillis) / 1000.0f;
  lastImuSampleMillis = currentMillis;
  imuYawDeg += gz * dtSeconds;
  while(imuYawDeg >= 360.0f) {
    imuYawDeg -= 360.0f;
  }
  while(imuYawDeg < 0.0f) {
    imuYawDeg += 360.0f;
  }

  Serial.print(currentMillis);
  Serial.print(",imu,");
  Serial.print(ax, 4);
  Serial.print(",");
  Serial.print(ay, 4);
  Serial.print(",");
  Serial.print(az, 4);
  Serial.print(",");
  Serial.print(gx, 3);
  Serial.print(",");
  Serial.print(gy, 3);
  Serial.print(",");
  Serial.print(gz, 3);
  Serial.print(",");
  Serial.println(imuYawDeg, 2);
}

void distanceSensingLoop() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastDistanceMillis < DISTANCE_INTERVAL_MS) {
    return;
  }
  lastDistanceMillis = currentMillis;

  float dist = readDistance();

  // Clamp distance to reasonable range (2cm to 400cm for HC-SR04)
  if(dist < 2) dist = 2;
  if(dist > 400) dist = 400;

  lastDistance = dist;
  queue[qindex++ % K] = dist;
  averageDistance = getAverage();

  lcd.clear();
  String text = "Dist: ";
  text += lastDistance;
  text += "cm";
  lcd.drawString(0, 0, text);

  text = "Avg: ";
  text += averageDistance;
  text += "cm";
  lcd.drawString(0, 18, text);

  text = "Speed: ";
  text += dutyCycle;
  text += "%";
  lcd.drawString(0, 36, text);
  lcd.display();

  Serial.print(currentMillis);
  Serial.print(",distance,");
  Serial.println(averageDistance, 2);
}

void controlLoop() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastControlMillis < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlMillis = currentMillis;

  readSerialCommands();
  updateActionCommand(currentMillis);

}

void loop() {
  controlLoop();
  imuLoop();
  distanceSensingLoop();
}
