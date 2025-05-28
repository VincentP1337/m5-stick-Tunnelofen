#include <M5Unified.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <math.h>
#include <ArduinoJson.h>

const char* ssid = "IIoT-Students";
const char* password = "b6H948!%^%$9$P";
const char* mqtt_server = "192.168.150.10";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

volatile bool mqttMessageReceived = false;
String lastTemperature = "-";
String lastVibrationMsg = "-";

#define VIB_PIN 26

// Callback für empfangene MQTT-Nachrichten
void callback(char* topic, byte* payload, unsigned int length) {
  mqttMessageReceived = true;
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (String(topic) == "tunnelofen/data") {
    // JSON parsen und currentTemp extrahieren
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (!error && doc.containsKey("currentTemp")) {
      lastTemperature = String(doc["currentTemp"].as<float>(), 1); // eine Nachkommastelle
    } else {
      lastTemperature = "-";
    }
  } else if (String(topic) == "m5stick/vibration") {
    lastVibrationMsg = message;
  }
}

void setup() {
  M5.begin();
  Serial.begin(115200);

  pinMode(VIB_PIN, OUTPUT);

  // Vibrationsmotor für 500ms einschalten (korrekt für M5StickC Plus 2)
  M5.Power.setVibration(1); // Einschalten
  delay(500);
  M5.Power.setVibration(0); // Ausschalten

  // Display initialisieren und Hintergrund löschen
  M5.Display.setRotation(1);
  M5.Display.fillScreen(BLACK);

  // WiFi verbinden
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(RED, BLACK);
    M5.Display.print("WiFi verbinden...");
  }
  M5.Display.fillScreen(BLACK);

  // MQTT einrichten
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
}

void loop() {
  static String lastDisplayTemp = "";
  static int lastBatPercent = -1;
  static bool lastWifi = false;
  static bool lastMqtt = false;

  M5.Display.setTextSize(2);

  // WiFi Status prüfen
  bool wifiConnected = WiFi.status() == WL_CONNECTED;

  // MQTT Status prüfen und verbinden falls nötig
  if (!mqttClient.connected() && wifiConnected) {
    if (mqttClient.connect("M5StickCClient")) {
      mqttClient.subscribe("tunnelofen/data");
      mqttClient.subscribe("m5stick/vibration");
    }
  }
  mqttClient.loop();

  // Batterieanzeige (Prozent)
  float batVoltage = M5.Power.getBatteryVoltage();
  int batPercent = (int)((batVoltage - 3.3) / (4.2 - 3.3) * 100.0);
  if (batPercent > 100) batPercent = 100;
  if (batPercent < 0) batPercent = 0;

  // Nur neu zeichnen, wenn sich etwas geändert hat
  if (lastTemperature != lastDisplayTemp ||
      lastBatPercent != batPercent ||
      lastWifi != wifiConnected ||
      lastMqtt != mqttClient.connected() ||
      mqttMessageReceived) {

    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);

    M5.Display.setTextColor(wifiConnected ? GREEN : RED, BLACK);
    M5.Display.printf("WiFi: %s\n", wifiConnected ? "Verbunden" : "Nicht verbunden");

    M5.Display.setTextColor(mqttClient.connected() ? GREEN : RED, BLACK);
    M5.Display.printf("MQTT: %s\n", mqttClient.connected() ? (mqttMessageReceived ? "Empfangen" : "Verbunden") : "Nicht verbunden");

    M5.Display.setTextColor(WHITE, BLACK);
    float tempValue = lastTemperature.toFloat();
    M5.Display.printf("Temp: %.1f\n", tempValue);

    // Vibration-Topic anzeigen
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("Vibration: %s\n", lastVibrationMsg.c_str());

    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.printf("Batterie: %d %%\n", batPercent);

    lastDisplayTemp = lastTemperature;
    lastBatPercent = batPercent;
    lastWifi = wifiConnected;
    lastMqtt = mqttClient.connected();
    mqttMessageReceived = false;
  }

  static unsigned long lastVibrationCheck = 0;
  unsigned long now = millis();

  // Alle 3 Sekunden Vibration prüfen und ggf. auslösen
  if (now - lastVibrationCheck >= 3000) {
    lastVibrationCheck = now;

    // 3 Sekunden "Dauer"-Vibration mit unterschiedlicher Intensität
    unsigned long vibStart = millis();
    if (lastVibrationMsg == "green") {
      // Sanft: kurze Pulse, lange Pausen
      while (millis() - vibStart < 3000) {
        digitalWrite(VIB_PIN, HIGH);
        delay(30);
        digitalWrite(VIB_PIN, LOW);
        delay(120);
      }
      // Danach: 1x kurz
      digitalWrite(VIB_PIN, HIGH); delay(100); digitalWrite(VIB_PIN, LOW); delay(100);
    } else if (lastVibrationMsg == "yellow") {
      // Mittel: mittlere Pulse/Pausen
      while (millis() - vibStart < 3000) {
        digitalWrite(VIB_PIN, HIGH);
        delay(80);
        digitalWrite(VIB_PIN, LOW);
        delay(80);
      }
      // Danach: 2x kurz
      for (int i = 0; i < 2; i++) {
        digitalWrite(VIB_PIN, HIGH); delay(100); digitalWrite(VIB_PIN, LOW); delay(100);
      }
    } else if (lastVibrationMsg == "red") {
      // Stark: lange Pulse, kurze Pausen
      while (millis() - vibStart < 3000) {
        digitalWrite(VIB_PIN, HIGH);
        delay(150);
        digitalWrite(VIB_PIN, LOW);
        delay(30);
      }
      // Danach: 3x kurz
      for (int i = 0; i < 3; i++) {
        digitalWrite(VIB_PIN, HIGH); delay(100); digitalWrite(VIB_PIN, LOW); delay(100);
      }
    }
  }

  // Knopf A prüfen und 5-fach vibrieren
  M5.update(); // Buttons aktualisieren
  if (M5.BtnA.wasPressed()) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(VIB_PIN, HIGH);
      delay(100);
      digitalWrite(VIB_PIN, LOW);
      delay(100);
    }
  }

  M5.update();
  delay(50); // sehr kurzer Delay, damit das System nicht überlastet wird
}