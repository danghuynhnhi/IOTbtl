#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// ========== PIN DEFINITION ==========
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define TRIG_PIN 19
#define ECHO_PIN 23
#define SERVO_PIN 26
#define FAN 2
#define FAN_PWM_CHANNEL 4

#define PIR_PIN 27
#define IR_PIN 32
#define LED_PIN 33
#define NUM_LEDS 4

// ========== NETWORK CONFIG ==========
constexpr char WIFI_SSID[] = "CAO coffee";
constexpr char WIFI_PASSWORD[] = "cao71phamvanxao";

constexpr char TOKEN[] = "8zxpedrndyzdhk5jb3x0";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024;
constexpr uint16_t telemetrySendInterval = 5000;
constexpr uint16_t ultrasonicCheckInterval = 300;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// ========== OBJECTS ==========
DHT20 dht20;
Servo myServo;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ========== GLOBAL VARIABLES ==========
int fanSpeed = 0;
bool attributesSent = false;
unsigned long lastSendTime = 0;
unsigned long lastUltrasonicCheck = 0;

float distance = 0;
unsigned long duration;
unsigned long doorOpenTime = 0;
bool isDoorOpen = false;

// ========== RPC: FAN SPEED ==========
void processFanSpeedChange(JsonVariantConst const &request, JsonDocument &response) {
  if (!request["speed"]) {
    Serial.println("⚠️ Request không có trường 'speed'");
    return;
  }

  int speed = request["speed"].as<int>();
  if (speed < 0 || speed > 100) {
    Serial.println("⚠️ Giá trị speed ngoài phạm vi 0-100");
    return;
  }

  fanSpeed = speed;
  int pwmValue = map(fanSpeed, 0, 100, 0, 255);
  ledcWrite(FAN_PWM_CHANNEL, pwmValue);
  Serial.printf("✅ Fan speed updated: %d%% (PWM: %d)\n", fanSpeed, pwmValue);

  response["status"] = "ok";
  response["fanSpeed"] = fanSpeed;
}

RPC_Callback fanSpeedCallback("fanSpeed", processFanSpeedChange);

// ========== INIT ==========
void InitWiFi() {
  Serial.println("🔌 Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
}

// ========== DOOR ==========
void openDoor() {
  if (!isDoorOpen) {
    myServo.write(120);
    isDoorOpen = true;
    doorOpenTime = millis();
    Serial.println("🚪 Mở cửa");
  }
}

void updateDoor() {
  if (isDoorOpen && (millis() - doorOpenTime >= 5000)) {
    myServo.write(0);
    isDoorOpen = false;
    Serial.println("🚪 Đóng cửa");
  }
}

// ========== SENSOR FUNCTIONS ==========
void checkUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = duration * 0.034 / 2;

  Serial.printf("📏 Khoảng cách: %.2f cm\n", distance);
  if (distance > 0 && distance < 15) {
    Serial.println("🚶 Vật thể gần - mở cửa");
    openDoor();
  }
}

void updatePIR() {
  bool motion = digitalRead(PIR_PIN) == HIGH;
  tb.sendTelemetryData("pir_motion", motion);
  Serial.println(String("👀 PIR: ") + (motion ? "Phát hiện" : "Không"));

  strip.setPixelColor(1, motion ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0));
  strip.show();
}

void updateIR() {
  bool isDark = digitalRead(IR_PIN) == LOW;
  tb.sendTelemetryData("ir_dark", isDark);
  Serial.println(String("🌙 IR ánh sáng: ") + (isDark ? "Tối" : "Sáng"));

  strip.setPixelColor(2, isDark ? strip.Color(255, 255, 0) : strip.Color(0, 0, 0));
  strip.show();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(PIR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);

  strip.begin();
  strip.show(); // Tắt tất cả LED

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 544, 2400);
  myServo.write(0);

  ledcSetup(FAN_PWM_CHANNEL, 5000, 8);
  ledcAttachPin(FAN, FAN_PWM_CHANNEL);

  InitWiFi();
}

// ========== LOOP ==========
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
  InitWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("📡 Đợi 5 giây rồi thử lại...");
    delay(5000);
    return;  // Thoát loop() tạm, tránh xử lý MQTT hay sensor
  }
}


  if (!tb.connected()) {
    Serial.printf("🔌 Connecting to ThingsBoard at %s...\n", THINGSBOARD_SERVER);
    if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("✅ MQTT đã kết nối ThingsBoard!");

      if (!attributesSent) {
        tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        attributesSent = true;
      }

      tb.RPC_Subscribe(fanSpeedCallback);
    } else {
      Serial.println("❌ MQTT kết nối thất bại!");
      delay(5000);
      return;
    }
  }

  // === Cập nhật toàn bộ dữ liệu mỗi 5 giây ===
  if (millis() - lastSendTime >= telemetrySendInterval) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.printf("🌡️ Nhiệt độ: %.2f°C | 💧 Độ ẩm: %.2f%%\n", temperature, humidity);
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);

      // LED đỏ nếu nhiệt độ > 30°C
      strip.setPixelColor(0, temperature > 30.0 ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0));
    } else {
      Serial.println("⚠️ Lỗi đọc DHT20");
    }

    // Đọc siêu âm
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = duration * 0.034 / 2;
    tb.sendTelemetryData("distance", distance);
    Serial.printf("📏 Khoảng cách: %.2f cm\n", distance);
    if (distance > 0 && distance < 15) {
      Serial.println("🚶 Vật thể gần - mở cửa");
      openDoor();
    }

    // Đọc PIR
    bool motion = digitalRead(PIR_PIN) == HIGH;
    tb.sendTelemetryData("pir_motion", motion);
    Serial.println(String("👀 PIR: ") + (motion ? "Phát hiện" : "Không"));
    strip.setPixelColor(1, motion ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0));

    // Đọc cảm biến ánh sáng IR
    bool isDark = digitalRead(IR_PIN) == LOW;
    tb.sendTelemetryData("ir_dark", isDark);
    Serial.println(String("🌙 IR ánh sáng: ") + (isDark ? "Tối" : "Sáng"));
    strip.setPixelColor(2, isDark ? strip.Color(255, 255, 0) : strip.Color(0, 0, 0));

    // Cập nhật RSSI WiFi
    tb.sendTelemetryData("rssi", WiFi.RSSI());

    tb.sendTelemetryData("fanSpeed", fanSpeed);

    strip.show();

    lastSendTime = millis();
  }

  // Cập nhật cửa
  updateDoor();

  // Duy trì kết nối ThingsBoard
  tb.loop();
}
