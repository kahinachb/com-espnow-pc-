## Guide pour comprendre les différents bus et protocoles de communication utilisés  

=> Voir photo de chaîne de communication  

Sur chaque patch se trouve un microcontrôleur STM32, ce microcontrôleur peut communiquer vers un autre composant en liaison série avec les broches RX/TX.  
Nous utilisons cette liaison RX/TX pour convertir cette liaison série en liaison RS485 grâce à un convertisseur UART / RS485.  
Ce bus RS485 nous permet d'avoir une communication multipoint, rapide, le tout sur une longue distance.  

4 STM32 des 4 patchs ainsi que l'ESP32 du module de communication se trouvent sur le bus RS485. C'est donc 5 composants qui communiquent entre eux grâce à ce bus.  
Néamoins, il faut faire attention à comment nous programmons avec ce bus RS485, en effet quand un des composants "parle" (ou écrit) sur ce bus, il faut que tous les autres composants soient en mode "écoute".  

## Code exemple d'envoi d'un nombre (pour STM32)
``` c
LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3); //DE : high, RE : low.1  
Serial.write(nombre);  
Serial.flush();  
LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3); 
```

La première ligne sert à mettre le convertisseur UART / RS485 en mode écriture (il faut veiller à ce que tous les autres composants soient en mode lecture).  
On utilise `Serial.write(nombre)` pour envoyer une donnée, ou bien `Serial.write(trame,tailleDeTrame)` pour envoyer une trame de données.  
'Serial.flush()' est utilisé pour s'assurer que les données sont envoyées sur la liaison série.  
Puis la dernière ligne sert à activer le mode lecture.  

## Alternative (pour ESP32)
```c
digitalWrite(DE_RE_PIN, HIGH); // Mode transmission  
Serial0.write(100);            // Octet spécial pour le top départ  
Serial0.flush(); // SERIAL2 POUR LE PIED DROIT ET SERIAL0 POUR LE PIED GAUCHE  
digitalWrite(DE_RE_PIN, LOW); // Mode réception  
```

## Comment les microcontrôleurs communiquent entre eux ?
Pour que le module de communication récupère toutes les données de la semelle pour ensuite les envoyer vers le PC, il envoie un signal "top départ" (ici un nombre sur un octet) sur le bus RS485, les 4 STM32 étant en mode écoute, ils commutent en mode écriture après avoir reçu le top départ.  
Mais attention : chaque STM32 possède un délai propre à leur numéro de patch, cela veut dire que c'est en premier le talon qui passe en mode écriture pour envoyer les données de la semelle, en un temps inférieur à 100µs. Après le délai imposé pour le deuxième patch, c'est à son tour d'envoyer ses données, ainsi de suite.  
Pour avoir une fréquence de communication de 200 Hz, le module de communication envoie son top départ toutes les 5000 µs.  

### Etapes lors de la communication
1. Les semelles sont allumées, rien ne se passe.  
2. L'ESP32 de réception est alimenté.  
3. 5 secondes plus tard, l'ESP32 de réception envoie aux 2 modules de communication un signal top départ (un octet).  
4. Après avoir reçu ce signal, les ESP32 des modules de communication peuvent commencer, ils envoient alors un signal top départ aux 4 patchs en même temps.  
5. Ces 4 patchs étaient en mode lecture, maintenant qu'ils ont tous reçu le signal, le patch du talon commute en mode écriture, envoie toutes ses données sur le bus RS485 et recommute en mode lecture.  
6. Le patch suivant écris ses données à son tour, et ainsi de suite pour les 4 patchs de la semelle, chaque patch attend un temps différent pour garder un bon ordre des trames reçues.  
7. Pendant que les 4 patchs écrivent leurs données, le module de communication récupère les 4 trames, et une fois les 4 trames récoltées il les envoie en sans-fil avec le protocole ESP-NOW vers le module de réception.  
8. Le module de communication recommence à envoyer le top départ à une fréquence précise, on a donc l'acquisition des données en continu.  
9. Le module de réception reçoit ses trames, les stocke et les écris directement sur le bus USB vers le PC.  
10. Sur le PC, les données qui arrivent sont directement enregistrées dans un fichier binaire.  
  
## Code d'envoi des données avec un délai propre au patch
```c
void loop() {  
  byte octet = 0;  
  while (octet != 100) { // Attente de la détection du top départ envoyé par le module de com  
    if (Serial.available()) { // Si des données disponibles  
      octet = Serial.read(); // On lit l'octet  
    }
  }
  delayMicroseconds(300 + 100 * (patch)); // Délai imposé pour respecter la commutation du mode transmission au mode réception  
  read_send_data();  
}  
```
Le module de communication, après avoir reçu les 4 trames de données (une de chaque patch), envoie d'un coup les 4 trames en sans-fil vers un autre ESP32 connecté au PC grâce au protocole ESP-NOW.    

## Trame d'une semelle
La trame d'une semelle entière reçue sur le PC se décompose sous cette forme :  
signal-ID1(talon)-ID2-CPT-F1-F2-F3-F4-accX-accY-accZ-gyX-gyY-gyZ-press-temp-x0-x1-y0-y1-z0-z1-xyz_ext-  
ID1(patch 2) -ID2-CPT-F1-F2-F3-F4-accX-accY-accZ-gyX-gyY-gyZ-press-temp-x0-x1-y0-y1-z0-z1-xyz_ext-  
ID1(patch 3)-ID2-CPT-F1-F2-F3-F4-accX-accY-accZ-gyX-gyY-gyZ-press-temp-x0-x1-y0-y1-z0-z1-xyz_ext-  
ID1(patch 4)-ID2-CPT-F1-F2-F3-F4-accX-accY-accZ-gyX-gyY-gyZ-press-temp-x0-x1-y0-y1-z0-z1-xyz_ext-timestamp  
  
Où :  
`signal` (uint8_t) correspond au signal de synchro envoyé depuis le programme python `envoiKeyboard.py` 
`ID1` et `ID2` (2x uint8_t) les identifiants de la trame  
`CPT` (uint8_t) le compteur interne d'un patch  
`F1`, `F2`, `F3` et `F4` (4x uint16_t) les forces brutes des capteurs de force
`accX`, `accY`, `accZ`, `gyX`, `gyY` et `gyZ` (6x int16_t) les données brutes de l'IMU  
`press` et `temp` (2x 3 octets, donc 2x 24 bits, attention à la conversion) la pression atmosphérique brute et la température brute du baromètre  
`x0`, `x1`, `y0`, `y1`, `z0`, `z1` et `xyz_ext` (7x uint8_t) les données du magnétomètre  
`timestamp` (uint32_t) le temps écoulé entre 2 réceptions de trame d'une semelle entière  
  
La trame entière d'une semelle fait donc  1+36x4+4 =  149 octets.  
Dans 1+36x4+4, 1 correspond à l'octet du signal du programme python, le 36x4 correspond aux 4 trames des patchs et le +4 correspond à l'uint32_t timestamp  

## Identifiants de trame
Les identifiants de trame de chaque patch possède en début de trame une paire d'identifiants (2 octets).  
Cette paire est sous la forme ID1, ID2.  
Patch 1 talon gauche : ID1 = 0x01, ID2 = 0x02  
Patch 2 (après talon): ID1 = 0x03, ID2 = 0x04  
Patch 3 : ID1 = 0x05, ID2 = 0x06  
Patch 4 orteils gauche : ID1 = 0x07, ID2 = 0x08  
  
Patch 1 talon droit : ID1 = 0x09, ID2 = 0x0A  
Patch 2 (après talon) : ID1 = 0x0B, ID2 = 0x0C  
Patch 3 : ID1 = 0x0D, ID2 = 0x0E  
Patch 4 orteils droit : ID1 = 0x0F, ID2 = 0x10 (16)
