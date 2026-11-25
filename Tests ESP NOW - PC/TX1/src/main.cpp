#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x54, 0x32, 0x04, 0x21, 0x8E, 0x44}; // Adresse MAC du récepteur
unsigned long lastMicros = 0;
unsigned long lastMillisCharge = 0;
const unsigned long interval = 3333; // en µs, 2000µs correspond à 500Hz, 10000µs correspond à 100Hz
const unsigned long intervalCharge = 10000; // en ms 
bool modeRecharge = false;
typedef struct struct_message {
  uint8_t entete1;
  uint8_t entete2;
  uint8_t bytes[30];

} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info,
                esp_now_send_status_t status)
{
  (void)tx_info;
  (void)status;
}

void setup() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return;
  }
  myData.entete1 = 102;
  myData.entete2 = 103;
}

void loop() {

  unsigned long newTime = micros();
  unsigned long newTimeCharge = millis();
  static uint8_t nb = 0;
  if (newTime - lastMicros >= interval && !modeRecharge) {
    lastMicros = newTime;
    for (int i = 0; i < 30; i++) {
      myData.bytes[i] = nb;
    }
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    // result permet de vérifier le bon envoi des données
    nb++;
  }

  if (newTimeCharge-lastMillisCharge>=intervalCharge){
    lastMillisCharge = newTimeCharge;
    uint32_t VCC_value = 0;
    for(int i = 0; i < 16; i++) {
      VCC_value += analogReadMilliVolts(A0); // Read and accumulate ADC voltage
    }
    float VCC_valueF = 2 * VCC_value / 16 / 1000.0;     // Adjust for 1:2 divider and convert to volts
    if (VCC_valueF>2.5){
      modeRecharge = true;
    }
    else{
      modeRecharge = false;
    }
  }
}