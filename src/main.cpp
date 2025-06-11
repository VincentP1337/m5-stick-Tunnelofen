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
String lastGeofenceStatus = "OUT"; // Status für Geofencing (IN/OUT)
String lastGeofenceRaw = "-"; // Rohe MQTT-Nachricht für Debug
String lastAreaId = "-"; // Area ID für Debug

#define VIB_PIN 26

// Vibrations-State-Machine Variablen
bool vibrationActive = false;
unsigned long vibrationStartTime = 0;
unsigned long lastVibrationToggle = 0;
bool vibrationMotorOn = false;
String currentVibrationLevel = "";
int vibrationPhase = 0; // 0 = Hauptvibration, 1 = Kurze Pulse

// Callback für empfangene MQTT-Nachrichten
void callback(char* topic, byte* payload, unsigned int length) {
  mqttMessageReceived = true;
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }  if (String(topic) == "tunnelofen/data") {
    // JSON parsen und currentTemp extrahieren
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (!error && doc.containsKey("currentTemp")) {
      lastTemperature = String(doc["currentTemp"].as<float>(), 1); // eine Nachkommastelle
    } else {
      lastTemperature = "-";
    }
  } else if (String(topic) == "m5stick/vibration") {
    lastVibrationMsg = message;  } else if (String(topic) == "zigpos/geofence") {
    lastGeofenceRaw = message; // Rohe Nachricht für Debug speichern
    // JSON parsen für Geofencing
    StaticJsonDocument<256> geoDoc;
    DeserializationError geoError = deserializeJson(geoDoc, message);
    if (!geoError) {
      // Area ID für Debug speichern
      lastAreaId = geoDoc["areaId"].as<String>();
      
      // Alle Geofence-Events verarbeiten (nicht nur Elena Stoll)
      lastGeofenceStatus = geoDoc["eventType"].as<String>(); // "IN" oder "OUT"
    }
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
      mqttClient.subscribe("zigpos/geofence");
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
    M5.Display.printf("Temp: %.1f\n", tempValue);    // Vibration-Topic anzeigen
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.printf("Vibration: %s\n", lastVibrationMsg.c_str());

    // Geofencing-Status anzeigen
    M5.Display.setTextColor(lastGeofenceStatus == "IN" ? RED : GREEN, BLACK);
    M5.Display.printf("Geofence: %s\n", lastGeofenceStatus == "IN" ? "GEFAHR!" : "SICHER");    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.printf("Batterie: %d %%\n", batPercent);

    // Debug: Geofence-Status anzeigen
    M5.Display.setTextColor(ORANGE, BLACK);
    M5.Display.printf("Debug: %s\n", lastGeofenceStatus.c_str());

    lastDisplayTemp = lastTemperature;
    lastBatPercent = batPercent;
    lastWifi = wifiConnected;
    lastMqtt = mqttClient.connected();
    mqttMessageReceived = false;
  }
  static unsigned long lastVibrationCheck = 0;
  unsigned long now = millis();
  
  // Alle 3 Sekunden prüfen ob neue Vibration gestartet werden soll
  if (now - lastVibrationCheck >= 3000) {
    lastVibrationCheck = now;

    // Nur vibrieren wenn Geofencing-Status "IN" ist und Vibrationsmodus aktiv
    if (lastGeofenceStatus == "IN" && (lastVibrationMsg == "green" || lastVibrationMsg == "yellow" || lastVibrationMsg == "red")) {
      // Neue Vibration starten
      vibrationActive = true;
      vibrationStartTime = now;
      lastVibrationToggle = now;
      vibrationMotorOn = false;
      currentVibrationLevel = lastVibrationMsg;
      vibrationPhase = 0; // Hauptvibration starten
    }
  }

  // Vibrations-State-Machine (non-blocking)
  if (vibrationActive) {
    unsigned long vibElapsed = now - vibrationStartTime;
    unsigned long toggleElapsed = now - lastVibrationToggle;
    
    if (vibrationPhase == 0) { // Hauptvibration (3 Sekunden)
      if (vibElapsed < 3000) {
        // Verschiedene Vibrationsmuster je nach Level
        if (currentVibrationLevel == "green") {
          // Sanft: 30ms an, 120ms aus
          if (!vibrationMotorOn && toggleElapsed >= 120) {
            digitalWrite(VIB_PIN, HIGH);
            vibrationMotorOn = true;
            lastVibrationToggle = now;
          } else if (vibrationMotorOn && toggleElapsed >= 30) {
            digitalWrite(VIB_PIN, LOW);
            vibrationMotorOn = false;
            lastVibrationToggle = now;
          }
        } else if (currentVibrationLevel == "yellow") {
          // Mittel: 80ms an, 80ms aus
          if (!vibrationMotorOn && toggleElapsed >= 80) {
            digitalWrite(VIB_PIN, HIGH);
            vibrationMotorOn = true;
            lastVibrationToggle = now;
          } else if (vibrationMotorOn && toggleElapsed >= 80) {
            digitalWrite(VIB_PIN, LOW);
            vibrationMotorOn = false;
            lastVibrationToggle = now;
          }
        } else if (currentVibrationLevel == "red") {
          // Stark: 150ms an, 30ms aus
          if (!vibrationMotorOn && toggleElapsed >= 30) {
            digitalWrite(VIB_PIN, HIGH);
            vibrationMotorOn = true;
            lastVibrationToggle = now;
          } else if (vibrationMotorOn && toggleElapsed >= 150) {
            digitalWrite(VIB_PIN, LOW);
            vibrationMotorOn = false;
            lastVibrationToggle = now;
          }
        }
      } else {
        // Hauptvibration beendet, zu kurzen Pulsen wechseln
        digitalWrite(VIB_PIN, LOW);
        vibrationMotorOn = false;
        vibrationPhase = 1;
        vibrationStartTime = now; // Neue Startzeit für Pulse
        lastVibrationToggle = now;
      }
    } else if (vibrationPhase == 1) { // Kurze Pulse
      int pulseCount = (currentVibrationLevel == "green") ? 1 : 
                      (currentVibrationLevel == "yellow") ? 2 : 3;
      int currentPulse = (int)(vibElapsed / 200); // Jeder Pulse dauert 200ms (100ms an + 100ms aus)
      
      if (currentPulse < pulseCount) {
        int pulsePhase = (int)(vibElapsed % 200);
        if (pulsePhase < 100 && !vibrationMotorOn) {
          digitalWrite(VIB_PIN, HIGH);
          vibrationMotorOn = true;
        } else if (pulsePhase >= 100 && vibrationMotorOn) {
          digitalWrite(VIB_PIN, LOW);
          vibrationMotorOn = false;
        }
      } else {
        // Alle Pulse beendet
        digitalWrite(VIB_PIN, LOW);
        vibrationActive = false;
        vibrationMotorOn = false;
      }
    }
  }
  // Knopf A prüfen und 5-fach vibrieren (non-blocking)
  M5.update(); // Buttons aktualisieren
  static bool buttonTestActive = false;
  static unsigned long buttonTestStart = 0;
  static int buttonTestPulse = 0;
  
  if (M5.BtnA.wasPressed() && !buttonTestActive) {
    buttonTestActive = true;
    buttonTestStart = millis();
    buttonTestPulse = 0;
  }
  
  if (buttonTestActive) {
    unsigned long buttonElapsed = millis() - buttonTestStart;
    int currentPulse = (int)(buttonElapsed / 200); // Jeder Pulse dauert 200ms
    
    if (currentPulse < 5) {
      int pulsePhase = (int)(buttonElapsed % 200);
      if (pulsePhase < 100 && buttonTestPulse != currentPulse) {
        digitalWrite(VIB_PIN, HIGH);
        buttonTestPulse = currentPulse;
      } else if (pulsePhase >= 100) {
        digitalWrite(VIB_PIN, LOW);
      }
    } else {
      digitalWrite(VIB_PIN, LOW);
      buttonTestActive = false;
    }
  }

  M5.update();
  delay(10); // Sehr kurzer Delay für bessere MQTT-Reaktion
}