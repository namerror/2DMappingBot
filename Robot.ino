#include <SSD1306Wire.h>
SSD1306Wire lcd(0x3c, SDA, SCL);

#define TRIG 5
#define ECHO 16

void setup() {
  //Serial.begin(9600);  // ← ADD THIS: start USB communication to laptop
  Serial.begin(115200); // Displays to Serial monitor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  ledcSetup(0, 10000, 20);
  lcd.init();
  lcd.flipScreenVertically();
  lcd.setFont(ArialMT_Plain_16);
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

float getAverage() {
  float sum = 0;
  for(int i = 0; i < K; i++) {  
    sum += queue[i];
  }
  return sum / K;
}

void loop() {
  float dist = readDistance();
  
  // Clamp distance to reasonable range (2cm to 400cm for HC-SR04)
  if(dist < 2) dist = 2;
  if(dist > 400) dist = 400;

  queue[qindex++ % K] = dist;

  // --- LCD DISPLAY (your original feature) ---
  lcd.clear();
  String text = "Dist: ";
  text += dist;
  text += "cm";
  lcd.drawString(0, 0, text);
  lcd.display();

  // --- NEW: Send distance to laptop over USB ---
  Serial.println(getAverage());  // ← THIS is what the laptop Python code reads

  // Wait 100ms between readings (adjust as needed)
  delay(100);
}