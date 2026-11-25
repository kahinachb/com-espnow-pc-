#include <WiFi.h>

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);  // important
  Serial.println(WiFi.macAddress());
}

void loop() {}
