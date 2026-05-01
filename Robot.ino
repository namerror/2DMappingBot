#include <SSD1306Wire.h>
#include <WiFi.h>

SSD1306Wire lcd(0x3c, SDA, SCL);

const char* ssid = "MA (2)";
const char* pass = "12345689";
IPAddress laptopIP(172, 20, 10, 2);    // laptop IP on hotspot
const uint16_t laptopPort = 8080;     // port laptop is listening on
WiFiClient client;

#define TRIG 5
#define ECHO 16

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  ledcSetup(0, 10000, 20);
  lcd.init();
  lcd.flipScreenVertically();
  lcd.setFont(ArialMT_Plain_16);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP = ");
  Serial.println(WiFi.localIP());
}

bool sendDistance(float value) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  if (!client.connected()) {
    client.stop();
    client.connect(laptopIP, laptopPort); // connect to laptop on hotspot
  }
  if (client.connected()) {
    client.print(value);
    client.print("\n");
    return true;
  }
  return false;
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

  // --- LCD DISPLAY ---
  lcd.clear();
  String text = "Dist: ";
  text += dist;
  text += "cm";
  lcd.drawString(0, 0, text);

  text = (WiFi.status() == WL_CONNECTED) ? "WiFi: Connected" : "WiFi: Disconnected";
  lcd.drawString(0, 20, text);

  float avg = getAverage();
  bool sent = sendDistance(avg);
  text = sent ? "Send: Done" : "Send: Error";
  lcd.drawString(0, 40, text);

  lcd.display();

  // --- NEW: Send distance to laptop over USB ---
  // Serial.println(getAverage());  // ← THIS is what the laptop Python code reads

  // --- NEW: Send averaged distance to laptop over WiFi station mode ---
  Serial.print("Distance sent: ");
  Serial.println(avg);

  // Wait 100ms between readings (adjust as needed)
  delay(100);
}