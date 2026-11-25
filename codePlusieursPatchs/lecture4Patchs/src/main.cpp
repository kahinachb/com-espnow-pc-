#include <Arduino.h>

/*
Code de debug sans communication ESPNOW, c'est directement le module de communication de la semelle qui communique avec le PC via le port série

La communication se passe de cette manière :
L'ESP du module de com envoie un "top départ" (un octet spécial) aux 4 patchs à une fréquence précise
A chaque fois que les 4 patchs reçoivent le top départ, ils envoient leurs données en respectant l'ordre patch 1 - 2 - 3 - 4 à l'aide d'un délai appliqué avant la transmission
Chaque trame est lue, mais l'envoi sans fil en ESP NOW se fait que lorsque les 4 trames des patchs sont reçues et stockées pour éviter de saturer la com ESP NOW tout en respectant
la fréquence de 250Hz
*/

#define DE_RE_PIN 1                               // GPIO1 pour contrôler DE/RE du transceiver RS485
const unsigned long intervalleTopDepart = 20000; // intervalle en µs qui correspond à la période entre chaque envoi du top départ qui permet de récupèrer les 4 trames respectives des patchs
// 4000 µs => 250 Hz

typedef struct struct_message
{ // Trame unique de 36 octets de data
  uint8_t bytes[36];
} struct_message;

typedef struct struct_combined_message
{                     // Grosse trame combinée de 4 trames (les 4 patchs) pour l'envoi en ESP NOW
  uint8_t bytes[144]; // 36 octets * 4 trames
} struct_combined_message;

struct_combined_message combinedData;
int trameCount = 0; // Compteur pour le nombre de trames stockées
int32_t mx, my, mz;

void envoiTopDepart() // Fonction pour l'envoi du top départ aux 4 patchs
{
  static int oldTime = 0;
  int newTime = micros();
  if (newTime - oldTime >= intervalleTopDepart)
  { // Envoi du top départ à 250 Hz (période 4000µs)
    oldTime = newTime;
    digitalWrite(DE_RE_PIN, HIGH); // Mode transmission
    Serial2.write(100);            // Octet spécial pour le top départ
    Serial2.flush();               // SERIAL2 POUR LE PIED DROIT ET SERIAL0 POUR LE PIED GAUCHE
    digitalWrite(DE_RE_PIN, LOW);  // Mode réception
  }
}

void setup()
{
  Serial.begin(2000000);
  Serial0.begin(2000000);
  Serial2.begin(2000000);
  pinMode(DE_RE_PIN, OUTPUT); // Configurer GPIO1 comme sortie
  delay(100);
}

void loop()
{
  // Serial.println("waiting");
  static int cpt;
  if (Serial0.available()) // Si il y a des données dispo à la lecture et que l'ESP n'est pas branché au PC
  {                        // On lit les données et on les stocke
    byte headers[2];       // Récupère l'entête (2 octets)
    Serial0.readBytes(headers, 2);

    int trameType = -1;
    // DECOMMENTER / COMMENTER CE QUE VOUS VOULEZ 
    
    if (headers[0] == 1 && headers[1] == 2)
      trameType = 0; // Identification du patch
     if (headers[0] == 3 && headers[1] == 4)
      trameType = 1;
     if (headers[0] == 5 && headers[1] == 6)
      trameType = 2;
     if (headers[0] == 7 && headers[1] == 8)
      trameType = 3;
      cpt++;
    
    if (headers[0] == 9 && headers[1] == 10)
     trameType = 0; // Identification du patch
    if (headers[0] == 11 && headers[1] == 12)
     trameType = 1;
   if (headers[0] == 13 && headers[1] == 14)
     trameType = 2;
    if (headers[0] == 15 && headers[1] == 16)
      trameType = 3;
    cpt++;
    if (trameType != -1)
    {
      byte data[34];
      Serial0.readBytes(data, 34); // Lecture des 34 octets de data après l'entête
      trameCount++;
      // COMMENTEZ / DECOMMENTEZ CE QUE VOUS VOULEZ POUR DEBUG
      /*
      uint32_t press = (data[21] << 16 | data[22] << 8) | data[23];
      uint32_t temp = (data[24] << 16 | data[25] << 8) | data[26];
      Serial.print(headers[0], HEX);
      Serial.print(" ");
      Serial.print(headers[1], HEX);
      Serial.print(" ");
      Serial.print(press);
      Serial.print(" ");
      Serial.println(temp);
      */

      Serial.print(headers[0], HEX);
      Serial.print(" ");
      Serial.print(headers[1], HEX);
      Serial.print(" ");

      int force1 = data[1] << 8 | data[2];
      int force2 = data[3] << 8 | data[4];
      int force3 = data[5] << 8 | data[6];
      int force4 = data[7] << 8 | data[8];
      int16_t accX = data[9] << 8 | data[10];
      int16_t accY = data[11] << 8 | data[12];
      int16_t accZ = data[13] << 8 | data[14];
      int16_t gyX = data[15] << 8 | data[16];
      int16_t gyY = data[17] << 8 | data[18];
      int16_t gyZ = data[19] << 8 | data[20];

      Serial.print(force1);
      Serial.print(" ");
      Serial.print(force2);
      Serial.print(" ");
      Serial.print(force3);
      Serial.print(" ");
      Serial.println(force4);
     
      Serial.print(" ");
      Serial.print(accX);
      Serial.print(" ");
      Serial.print(accY);
      Serial.print(" ");
      Serial.print(accZ);
      Serial.print(" ");
      Serial.print(gyX);
      Serial.print(" ");
      Serial.print(gyY);
      Serial.print(" ");
      Serial.print(gyZ);
      uint32_t press = (data[21] << 16 | data[22] << 8) | data[23];
      Serial.print(" ");
      Serial.print(press);
      Serial.println(" ");
      

      
      uint8_t x0 = data[27];
      uint8_t x1 = data[28];
      uint8_t y0 = data[29];
      uint8_t y1 = data[30];
      uint8_t z0 = data[31];
      uint8_t z1 = data[32];
      uint8_t xyz_ext = data[33];
      uint32_t x_raw = ((uint32_t)x0 << 10) | ((uint32_t)x1 << 2) | ((xyz_ext >> 6) & 0x03);
      uint32_t y_raw = ((uint32_t)y0 << 10) | ((uint32_t)y1 << 2) | ((xyz_ext >> 4) & 0x03);
      uint32_t z_raw = ((uint32_t)z0 << 10) | ((uint32_t)z1 << 2) | ((xyz_ext >> 2) & 0x03);
      mx = (int32_t)x_raw - 131072; // raw values
      my = (int32_t)y_raw - 131072;
      mz = (int32_t)z_raw - 131072;
      constexpr float MICROTESLA_PER_COUNT = 100.0f / 16384.0f;
      float fx = mx * MICROTESLA_PER_COUNT;
      float fy = my * MICROTESLA_PER_COUNT;
      float fz = mz * MICROTESLA_PER_COUNT;

      Serial.print("| X = ");
      Serial.print(fx);
      Serial.print(" | Y = ");
      Serial.print(fy);
      Serial.print(" | Z = ");
      Serial.println(fz);

      float norm = sqrt(fx * fx + fy * fy + fz * fz);
      Serial.print("Norme = ");
      Serial.println(norm);
      
    }
  }
  envoiTopDepart();
}
