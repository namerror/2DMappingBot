#include <SSD1306Wire.h>
#include <Arduino.h>

SSD1306Wire lcd(0x3c, SDA, SCL);

#define TRIG 5
#define ECHO 16
#define STBY A10
#define PWMA A17
#define PWMB A5
#define AIN1 A13 // A for left motor, B for right motor
#define AIN2 A14
#define BIN1 A15
#define BIN2 A16

const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_FREQUENCY = 10000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;
const unsigned long DISTANCE_INTERVAL_MS = 100;
const unsigned long CONTROL_INTERVAL_MS = 1000;

int dutyCycle = 0; // 0-100% duty cycle for motor speed control
unsigned long lastDistanceMillis = 0;
unsigned long lastControlMillis = 0;

void setup() {
  //Serial.begin(9600);  // ← ADD THIS: start USB communication to laptop
  Serial.begin(115200); // Displays to Serial monitor
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
  setMotorSpeed(0);
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

  Serial.println(averageDistance);
}

void controlLoop() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastControlMillis < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlMillis = currentMillis;

  moveForward();
  setMotorSpeed(50);
}

void loop() {
  distanceSensingLoop();
  controlLoop();
}
