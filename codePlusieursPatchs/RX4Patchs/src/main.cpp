#include <esp_now.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
// Code de réception pour l'ESP32 côté PC
// Il attend des données qui arrivent en ESP NOW puis les envoie directement au PC via le port série
// Un switch relié à D7 définit le bit de synchronisation. Son état est ajouté
// au flux envoyé au PC et recopié sur une sortie physique (GPIO2).

// Chaque message TX4Patchs contient les quatre trames de 36 octets et le
// timestamp de 4 octets. La file sépare la réception ESP-NOW de l'écriture
// USB afin que le callback et loop() ne modifient jamais le même buffer.
constexpr size_t ESP_NOW_PACKET_SIZE = 148;
constexpr size_t USB_RECORD_SIZE = ESP_NOW_PACKET_SIZE + 1;
constexpr size_t USB_TX_BUFFER_SIZE = 4096;
constexpr uint16_t RX_QUEUE_LENGTH = 32;

struct ReceivedPacket
{
  uint8_t data[ESP_NOW_PACKET_SIZE];
};

const int PIN = 2;         // Sortie optionnelle du signal de synchronisation vers la Bertec
const int SWITCH_PIN = D7; // Switch câblé entre D7 et GND
const unsigned long SWITCH_DEBOUNCE_MS = 20;
uint8_t broadcastAddressLeft[] = {0x8C, 0xBF, 0xEA, 0xCC, 0x9C, 0x04}; // Adresse MAC du récepteur
uint8_t broadcastAddressRight[] = {0x8C, 0xBF, 0xEA, 0xCC, 0xA8, 0xDC}; // Adresse MAC du récepteur

QueueHandle_t packetQueue = nullptr;
uint32_t droppedPacketCount = 0;

esp_now_peer_info_t peerInfoLeft;
esp_now_peer_info_t peerInfoRight;
bool topSynchro = false;
bool lastRawSwitchState = HIGH;
unsigned long lastSwitchChangeMs = 0;

bool configurerDebitEspNow(const uint8_t *peerAddress)
{
  esp_now_rate_config_t rateConfig = {};
  rateConfig.phymode = WIFI_PHY_MODE_11G;
  rateConfig.rate = WIFI_PHY_RATE_6M;
  rateConfig.ersu = false;
  rateConfig.dcm = false;

  return esp_now_set_peer_rate_config(peerAddress, &rateConfig) == ESP_OK;
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
  (void)tx_info;
  (void)status;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  (void)mac_addr;

  if (len != ESP_NOW_PACKET_SIZE || packetQueue == nullptr)
  {
    return;
  }

  ReceivedPacket packet;
  memcpy(packet.data, incomingData, ESP_NOW_PACKET_SIZE);

  // Le callback ESP-NOW ne doit jamais attendre l'écriture USB.
  if (xQueueSend(packetQueue, &packet, 0) != pdTRUE)
  {
    droppedPacketCount++;
  }
}

void updateSyncSignal()
{
  // INPUT_PULLUP : D7 à 3,3 V donne Signal = 1 ; D7 reliée à GND donne 0.
  const bool rawSwitchState = digitalRead(SWITCH_PIN);
  if (rawSwitchState != lastRawSwitchState)
  {
    lastRawSwitchState = rawSwitchState;
    lastSwitchChangeMs = millis();
  }

  if (millis() - lastSwitchChangeMs >= SWITCH_DEBOUNCE_MS)
  {
    topSynchro = (lastRawSwitchState == HIGH);
  }

  // Sortie optionnelle vers la Bertec ; elle n'influence pas le CSV.
  digitalWrite(PIN, topSynchro ? HIGH : LOW);
}

void writeUsbRecord(const ReceivedPacket &packet, bool syncSignal)
{
  // Un enregistrement PC est toujours indivisible du point de vue du format :
  // 1 octet Signal suivi des 148 octets reçus par ESP-NOW.
  uint8_t usbRecord[USB_RECORD_SIZE];
  usbRecord[0] = syncSignal ? 1 : 0;
  memcpy(&usbRecord[1], packet.data, ESP_NOW_PACKET_SIZE);

  // Serial.write() peut accepter moins d'octets que demandé lorsque son
  // tampon USB est momentanément plein. Continuer au même offset empêche de
  // créer un fragment avant de traiter le paquet suivant.
  size_t written = 0;
  while (written < USB_RECORD_SIZE)
  {
    const size_t count = Serial.write(
      &usbRecord[written],
      USB_RECORD_SIZE - written
    );
    if (count == 0)
    {
      delay(1);
      continue;
    }
    written += count;
  }
}

void setup()
{
  // Le tampon HWCDC par défaut (256 octets) contient moins de deux
  // enregistrements de 149 octets. Cette réserve absorbe les ralentissements
  // USB brefs avant que writeUsbRecord() ne doive attendre.
  if (Serial.setTxBufferSize(USB_TX_BUFFER_SIZE) != USB_TX_BUFFER_SIZE)
  {
    return;
  }
  Serial.begin(2000000); // Pour l'ESP32C3/C6
  // Serial.begin(921600); // Pour l'ESP WROOM 32
  pinMode(PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastRawSwitchState = digitalRead(SWITCH_PIN);
  topSynchro = (lastRawSwitchState == HIGH);
  digitalWrite(PIN, topSynchro ? HIGH : LOW);
  WiFi.mode(WIFI_STA);

  packetQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(ReceivedPacket));
  if (packetQueue == nullptr)
  {
    return;
  }

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfoLeft.peer_addr, broadcastAddressLeft, 6);
  memcpy(peerInfoRight.peer_addr, broadcastAddressRight, 6);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  peerInfoLeft.channel = 0;
  peerInfoLeft.encrypt = false;
  peerInfoRight.channel = 0;
  peerInfoRight.encrypt = false;
  if (esp_now_add_peer(&peerInfoLeft) != ESP_OK)
  {
    return;
  }
  if (esp_now_add_peer(&peerInfoRight) != ESP_OK)
  {
    return;
  }
  if (!configurerDebitEspNow(broadcastAddressLeft) ||
      !configurerDebitEspNow(broadcastAddressRight))
  {
    return;
  }
  delay(5000);
  uint8_t data[1];
  data[0]=100;
  esp_now_send(broadcastAddressLeft, (uint8_t *)&data, 1);
  esp_now_send(broadcastAddressRight, (uint8_t *)&data, 1);

}

void loop()
{
  // Lecture du switch indépendamment de l'arrivée des paquets ESP-NOW.
  updateSyncSignal();

  ReceivedPacket packet;
  while (xQueueReceive(packetQueue, &packet, 0) == pdTRUE)
  {
    writeUsbRecord(packet, topSynchro);
  }
}
