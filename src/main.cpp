#include <M5Unified.h>
#include <PubSubClient.h>
#include <Wifi.h>

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  M5.begin();
  Serial.begin(115200);
}

void loop() {
  delay(1000);
  auto timestamp = M5.Rtc.getTime();
  Serial.printf("Looped. Time: %02d:%02d:%02d\n", timestamp.hours,
                timestamp.minutes, timestamp.seconds);
  M5.update();
}