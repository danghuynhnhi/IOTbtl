#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define TRIG_PIN 19
#define ECHO_PIN 23
#define SERVO_PIN 26
#define FAN 2
#define FAN_PWM_CHANNEL 4 // tránh trùng channel với servo

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ESP32Servo.h>
#include <ArduinoJson.h>

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

constexpr char TOKEN[] = "8zxpedrndyzdhk5jb3x0";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200;
constexpr uint16_t telemetrySendInterval = 5000;
constexpr uint16_t ultrasonicCheckInterval = 300;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
Servo myServo;
int fanSpeed = 0;

bool attributesSent = false;
unsigned long lastSendTime = 0;
unsigned long lastUltrasonicCheck = 0;

float distance = 0;
unsigned long duration;

// Điều khiển cửa non-blocking
unsigned long doorOpenTime = 0;
bool isDoorOpen = false;

void processFanSpeedChange(JsonVariantConst const &request, JsonDocument &response) {
  Serial.print(F("Received RPC request: "));
  serializeJson(request, Serial);
  Serial.println();

  if (!request["speed"]) {
    Serial.println(F("⚠️ Request không có trường 'speed'"));
    return;
  }

  int speed = request["speed"].as<int>();
  if (speed < 0 || speed > 100) {
    Serial.println(F("⚠️ Giá trị speed ngoài phạm vi 0-100"));
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

void InitWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// 👉 Mở cửa (non-blocking)
void openDoor() {
  if (!isDoorOpen) {
    myServo.write(120);  // mở cửa
    isDoorOpen = true;
    doorOpenTime = millis();
    Serial.println("🚪 Mở cửa");
  }
}

// 👉 Cập nhật trạng thái cửa (đóng sau 5s)
void updateDoor() {
  if (isDoorOpen && (millis() - doorOpenTime >= 5000)) {
    myServo.write(0); // đóng cửa
    isDoorOpen = false;
    Serial.println("🚪 Đóng cửa");
  }
}

void checkUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  distance = duration * 0.034 / 2;

  Serial.print("📏 Khoảng cách đo: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance < 15) {
    Serial.println("🚶 Vật thể gần - mở cửa");
    openDoor();
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();

  myServo.setPeriodHertz(50); // tiêu chuẩn servo
  myServo.attach(SERVO_PIN, 544, 2400);
  myServo.write(0);

  ledcSetup(FAN_PWM_CHANNEL, 5000, 8); // 5kHz, độ phân giải 8-bit
  ledcAttachPin(FAN, FAN_PWM_CHANNEL);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    InitWiFi();
  }

  if (!tb.connected()) {
    Serial.printf("Connecting to ThingsBoard at %s...\n", THINGSBOARD_SERVER);
    if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("✅ Connected to ThingsBoard");
      if (!attributesSent) {
        tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        attributesSent = true;
      }
      tb.RPC_Subscribe(fanSpeedCallback);
    } else {
      Serial.println("❌ Failed to connect to ThingsBoard");
      delay(5000);
      return;
    }
  }

  // Gửi dữ liệu môi trường
  if (millis() - lastSendTime >= telemetrySendInterval) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.printf("🌡️ Temp: %.2f °C | 💧 Humidity: %.2f %%\n", temperature, humidity);
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    } else {
      Serial.println("⚠️ DHT20 read failed.");
    }

    tb.sendTelemetryData("rssi", WiFi.RSSI());
    lastSendTime = millis();
  }

  // Kiểm tra siêu âm
  if (millis() - lastUltrasonicCheck >= ultrasonicCheckInterval) {
    checkUltrasonic();
    lastUltrasonicCheck = millis();
  }

  // Cập nhật cửa
  updateDoor();

  tb.loop();
}
