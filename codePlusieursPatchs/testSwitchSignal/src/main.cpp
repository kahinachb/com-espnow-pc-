#include <Arduino.h>

// Switch câblé entre D7 et GND.
// Avec INPUT_PULLUP : D7 à 3,3 V donne Signal = 1 ; D7 à 0 V donne Signal = 0.
constexpr uint8_t SWITCH_PIN = D7;

void setup()
{
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.println("Test continu du switch sur D7");
}

void loop()
{
  const int rawState = digitalRead(SWITCH_PIN);
  const int signal = (rawState == HIGH) ? 1 : 0;
  Serial.printf("D7 = %d | Signal = %d\n", rawState, signal);
  delay(10);
}
