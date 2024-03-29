#include <Arduino.h>

//BLE
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "PubSubClient.h"
#include <WiFi.h>

//geluid
#include "AudioFileSourcePROGMEM.h" 
#include "AudioGeneratorMOD.h"
#include "AudioOutputI2S.h"

//geluid bestanden
#include "enigma.h"

//gebruik core 2
TaskHandle_t Task1;

AudioGeneratorMOD *mod;
AudioFileSourcePROGMEM *file;
AudioFileSourcePROGMEM *file2;
AudioOutputI2S *out;

static void startListeningToNFC();
static void handleCardDetected();


int gain = 0; // max 4
int rssi1;
int rssi2;
int rssi3;
const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
boolean readerDisabled = false;
int irqCurr;
int irqPrev;
bool keuzeFile = 1;

// WiFiClient espClient;
// PubSubClient client(espClient);

uint64_t cardidSticker;

bool stickerDetected = false;  //als er een sticker (1 van de 4) gedetecteerd is
bool monsterDetected = false;  //als sticker 1 gedetecteerd is
bool sticker2detected = false; //als sticker 2 gedetecteerd is
bool sticker3detected = false; //als sticker 3 gedetecteerd is
bool sticker4detected = false; //als sticker 4 gedetecteerd is

// This example uses the IRQ line, which is available when in I2C mode.
#define PN532_IRQ   4
#define PN532_RESET 5
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

//voor BLE
const char * sender1 = "7c:9e:bd:ed:58:1a"; //adres dat we willen
const char * sender2 = "7c:9e:bd:f4:10:32";
const char * sender3 = "7c:9e:bd:2a:fc:1e";
const char * addrSenders;
const char * addrSender1; //het adres dat we ontvangen
const char * addrSender2;
const char * addrSender3;
bool freeFlag = true;
bool freeFlag1 = true; //het adres die we krijgen is een pointer, dus we moeten het vrijgeven, vandaar deze flag
bool freeFlag2 = true;
bool freeFlag3 = true;
int scanTime = 1; //In seconds
BLEScan* pBLEScan;

//klasse voor BLE: hier wordt rssi en adres van de zender opgeslagen
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      addrSenders = strdup(advertisedDevice.getAddress().toString().c_str());
      if (strcmp(addrSenders, sender1) == 0) {
        addrSender1 = strdup(addrSenders);
        rssi1 = advertisedDevice.getRSSI();
        freeFlag1 = false;
        // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      if(strcmp(addrSenders, sender2) == 0) {
        addrSender2 = strdup(addrSenders);
        rssi2 = advertisedDevice.getRSSI();
        freeFlag2 = false;
        // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      if(strcmp(addrSenders, sender3) == 0) {
        addrSender3 = strdup(addrSenders);
        rssi3 = advertisedDevice.getRSSI();
        freeFlag3 = false;
        // Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      free((char*)addrSenders);
    }
};

//functies NFC/RFID
void startListeningToNFC() {
  // Reset our IRQ indicators
  irqPrev = irqCurr = HIGH;

  Serial.println("Present an ISO14443A Card ...");
  nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
}

void handleCardDetected() {
    uint8_t success = false;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

    // read the NFC tag's info
    success = nfc.readDetectedPassiveTargetID(uid, &uidLength);
    Serial.println(success ? "Read successful" : "Read failed (not a card?)");
    

    if (success) {
      // Display some basic information about the card
      //Serial.println("Found an ISO14443A card");
      //Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
      //Serial.print("  UID Value: ");
      Serial.print("Card ID HEX Value: ");//hierachter doorsturen

      nfc.PrintHex(uid, uidLength); //dit vervangen door eigen creatie

      if (uidLength == 7){
        //We zitten met zo een witte rfid sticker
        cardidSticker = uid[0];
        cardidSticker <<= 8;
        cardidSticker |= uid[1];
        cardidSticker <<= 8;
        cardidSticker |= uid[2];  
        cardidSticker <<= 8;
        cardidSticker |= uid[3]; 
        cardidSticker <<= 8;
        cardidSticker |= uid[4];
        cardidSticker <<= 8;
        cardidSticker |= uid[5];  
        cardidSticker <<= 8;
        cardidSticker |= uid[6];   
        Serial.print("cardidsticker ");
        Serial.print(cardidSticker);
        stickerDetected = true;
      }     
      timeLastCardRead = millis();
    }
    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
} 

//gebruik 2de core:
  //speaker kan niet op zelfde core als scanner
  //speaker kan niet op zelfde core als RFID
  //speaker moet dus in een task, constant checken of er iets moet afgespeeld worden
   //rfid gaat wel trager, maar is niet erg, want we moeten toch wachten tot de 
   //speaker klaar is met afspelen, en dat duurt even.
void Task1code( void * parameter) {
  for(;;) {
    if (mod->isRunning()) {
      if (!mod->loop()) {
          mod->stop();
        }
    } else {
      Serial.println("_________________geluid_________________");
      Serial.print("MOD done");
      Serial.print(" gain en audio file: (0 is blaffen): ");
      Serial.print(gain);
      Serial.print(" , ");
      Serial.println(keuzeFile);
      Serial.println("_________________geluid_________________");
      file->close();
      delete file;
      out = new AudioOutputI2S(0, 1);
      out->SetGain(gain);
      if(keuzeFile == 0){
        file = new AudioFileSourcePROGMEM(enigma_mod, sizeof(enigma_mod));
      } else {
        file = new AudioFileSourcePROGMEM(enigmacoin_mod, sizeof(enigmacoin_mod));
      }
      gain = 0;
      mod->begin(file, out);
      cardidSticker = 0;
    }
  }
}

void setup()
{
  Serial.begin(115200);

  //BLE:
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  audioLogger = &Serial;

  //geluid:
  file = new AudioFileSourcePROGMEM(enigma_mod, sizeof(enigma_mod) );
  file2 = new AudioFileSourcePROGMEM(enigmacoin_mod, sizeof(enigmacoin_mod) );
  out = new AudioOutputI2S(0, 1); 
  out->SetGain(0); //max gain is 4
  mod = new AudioGeneratorMOD();
  mod->SetBufferSize(1024);
  mod->SetSampleRate(44100);
  mod->SetStereoSeparation(32);
  mod->begin(file, out);

  //task:
  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

  
  //NFC:
  Serial.println("Begin NFC532 Scanning Software.");
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN532 board");
    while (1); // halt
  }
      // Got ok data, print it out!
  Serial.print("-------Found chip PN5"); Serial.print((versiondata>>24) & 0xFF, HEX); 
  Serial.print("   Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.print((versiondata>>8) & 0xFF, DEC);
  Serial.println("--------");
  // configure board to read RFID tags
  nfc.SAMConfig();
  startListeningToNFC();
}

void loop()
{ 
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("------Devices found: ");
  Serial.print(foundDevices.getCount());
  Serial.println(" Scan done!--------");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

  if (readerDisabled) {
    
    if (millis() - timeLastCardRead > DELAY_BETWEEN_CARDS) {
      readerDisabled = false;
      startListeningToNFC();
    }
  } else {
    irqCurr = digitalRead(PN532_IRQ);
    // When the IRQ is pulled low - the reader has got something for us.
    if (irqCurr == LOW && irqPrev == HIGH) {  
       handleCardDetected(); //cardidSticker wordt hier ingevuld
       readerDisabled = true;
    }
    irqPrev = irqCurr;
  }

  //hieronder de code voor de stickers/RFID's:
    //Eerst moet monster gedetecteerd worden, dan pas zoeken naar de andere drugszakjes
  if(stickerDetected == true && cardidSticker == 1138283286187137){
    stickerDetected = false;
    monsterDetected = true;
    Serial.println("heeft gesnuffeld");
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1392551377325440){
    //hier sticker 2 gedetecteerd
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    sticker2detected = true;
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1384850500963712){
    //hier sticker 3 gedetecteerd
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    sticker3detected = true;
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1377149624601984){
    //hier sticker 4 gedetecteerd
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    sticker4detected = true;
  }


  //hieronder de code voor de BLE, als er een BLE device gedetecteerd wordt, komt het in 1 van deze 3 if's:
  if(freeFlag1 == false){
    rssi1 = rssi1 + 30;
    rssi1 = (int) rssi1 / 4;
    Serial.print("sterkte signaal 1: ");
    Serial.println(rssi1);

    //voorwaarde om te blaffen:
    if(sticker2detected == false && monsterDetected == true && strcmp(sender1,addrSender1) == 0){
      Serial.println("stil BLAFFEN");
      if(cardidSticker != 0){ // niet gelijk aan nul => er is een sticker gedetecteerd
        keuzeFile = 1; //mario coin geluid
        gain = 3.9; 
      } else if (0 > rssi1 && rssi1 >= -7){
        keuzeFile = 0; //geblaf geluid
        gain = 3.9;
      }
    }
  }

  if(freeFlag2 == false){
    rssi2 = rssi2 + 30;
    rssi2 = (int) rssi2 / 4;
    Serial.print("sterkte signaal 2: ");
    Serial.println(rssi2);
    if(sticker3detected == false && sticker2detected == true && monsterDetected == true && strcmp(sender2,addrSender2) == 0){
      Serial.println("stil BLAFFEN");
      if(cardidSticker != 0){
        keuzeFile = 1;
        gain = 3.9; 
      } else if (0 > rssi2 && rssi2 >= -7){
        keuzeFile = 0; //0 = geblaf
        gain = 3.9;
      }
    }
  }

  if(freeFlag3 == false){
    rssi3 = rssi3 + 30;
    rssi3 = (int) rssi3 / 4;
    Serial.print("sterkte signaal 3: ");
    Serial.println(rssi3);

    if(sticker4detected == false && sticker3detected == true && sticker2detected == true && monsterDetected == true && strcmp(sender3,addrSender3) == 0){
      Serial.println("stil BLAFFEN");
      if(cardidSticker != 0){
        keuzeFile = 1; 
        gain = 3.9; 
      } else if (0 > rssi3 && rssi3 >= -7){
        keuzeFile = 0;
        gain = 3.9;
      }
    }
  }

  if(cardidSticker != 0){
    keuzeFile = 1;
    gain = 3.9; 
  }

  //variabelen op nul zetten:

  if(freeFlag1 == false){
    free((char*)addrSender1);
    freeFlag1 = true;
  }
  if(freeFlag2 == false){
    free((char*)addrSender2);
    freeFlag2 = true;
  }
  if(freeFlag3 == false){
    free((char*)addrSender3);
    freeFlag3 = true;
  }
}
