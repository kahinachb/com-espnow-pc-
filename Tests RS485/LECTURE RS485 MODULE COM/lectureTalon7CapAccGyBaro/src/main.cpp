#include <Arduino.h>
#define DE_RE_PIN 1 // GPIO1 pour contrôler DE/RE du transceiver RS485

struct SensorData {
  int16_t forces[4];
  int16_t acc[3];
  int16_t gyro[3];
  uint32_t pressure;
  byte counter;
};

void setup() {
  Serial.begin(2000000);   // Moniteur série pour débogage
  Serial0.begin(2000000);  // RS485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Mode réception initial
}

void receive_data() {
  if (Serial0.available() >= 26) {
    byte headers[2];
    Serial0.readBytes(headers, 2);
    if (headers[0] == 0x01 && headers[1] == 0x02) { // Vérifiez l'identifiant
      byte rawData[24];
      Serial0.readBytes(rawData, 24);

      SensorData data;
      data.counter = rawData[0];

      for (int i = 0; i < 4; i++) {
        data.forces[i] = (rawData[1 + i * 2] << 8) | rawData[2 + i * 2];
      }
      for (int i = 0; i < 3; i++) {
        data.acc[i] = (rawData[9 + i * 2] << 8) | rawData[10 + i * 2];
        data.gyro[i] = (rawData[15 + i * 2] << 8) | rawData[16 + i * 2];
      }
      data.pressure = (((uint32_t)rawData[21] << 16) | ((uint32_t)rawData[22] << 8)) | rawData[23];

      // Affichage des données
      Serial.print(headers[0],HEX);
      Serial.print(headers[1],HEX);
      Serial.print(" F1: "); Serial.print(data.forces[0]);
      Serial.print(" F2: "); Serial.print(data.forces[1]);
      Serial.print(" F3: "); Serial.print(data.forces[2]);
      Serial.print(" F4: "); Serial.print(data.forces[3]);
      Serial.print(" AcX: "); Serial.print(data.acc[0]);
      Serial.print(" AcY: "); Serial.print(data.acc[1]);
      Serial.print(" AcZ: "); Serial.print(data.acc[2]);
      Serial.print(" GyX: "); Serial.print(data.gyro[0]);
      Serial.print(" GyY: "); Serial.print(data.gyro[1]);
      Serial.print(" GyZ: "); Serial.print(data.gyro[2]);
      Serial.print(" Press :"); Serial.print(data.pressure);
      Serial.print(" Count: "); Serial.println(data.counter);
    }
  }
}

void loop() {
  receive_data();
  Serial0.write(5);
  Serial0.flush();
}
