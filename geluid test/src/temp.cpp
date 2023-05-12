// naam esp: esphond
// wachtwoord esp: esphond

#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include <WiFi.h>
// #include "OTAlib.h"

#include "AudioFileSourcePROGMEM.h" 
#include "AudioGeneratorMOD.h"
#include "AudioOutputI2S.h"

#include "enigma.h"

TaskHandle_t Task1;

AudioGeneratorMOD *mod;
AudioFileSourcePROGMEM *file;
AudioFileSourcePROGMEM *file2;
AudioOutputI2S *out;

//OTA
// OTAlib ota("NETGEAR68", "excitedtuba713");

static void startListeningToNFC();
static void handleCardDetected();

// Pins used for I2C IRQ
#define PN532_IRQ   4
#define PN532_RESET 5

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

WiFiClient espClient;
PubSubClient client(espClient);

uint64_t cardidSticker;
bool stickerDetected = false;
bool monsterDetected = false;
const char * addrSenders;
const char * addrSender1;
const char * addrSender2;
const char * addrSender3;
bool freeFlag = true;
bool freeFlag1 = true;
bool freeFlag2 = true;
bool freeFlag3 = true;

bool sticker2detected = false;
bool sticker3detected = false;
bool sticker4detected = false;


const char * sender1 = "7c:9e:bd:ed:58:1a";//TODO: als deze gevonden is, adres veranderen naar volgend drugszakje 7c:9e:bd:ed:58:1a 
const char * sender2 = "7c:9e:bd:ed:58:1a";
const char * sender3 = "7c:9e:bd:ed:58:1a";

#include <esp32-hal-ledc.h>

const int speakerPin = 25; // use any PWM-capable output pin
int toneFrequency = 0;  // set the tone frequency in Hz
int tellen;

// valid_tags = 
// 0x02 0x82 0x00 0x08 0x7B 0x2B 0xC3
// 0x04 0x9C 0x49 0x6A 0x99 0x5B 0x80 
// 0x69 0x42 0xA2 0xB8 
// 0xB3 0xF7 0xC6 0x02 
// 0x04 0x6B 0x0F 0xE2 0x50 0x5A 0x80
// 0xA9 0xAF 0xAE 0xC2 
// 0x04 0x07 0xCC 0x52 0xA8 0x58 0x81 

// This example uses the IRQ line, which is available when in I2C mode.
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      addrSenders = strdup(advertisedDevice.getAddress().toString().c_str());
      if (strcmp(addrSenders, sender1) == 0) {
        addrSender1 = strdup(addrSenders);
        rssi1 = advertisedDevice.getRSSI();
        freeFlag1 = false;
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      if(strcmp(addrSenders, sender2) == 0) {
        addrSender2 = strdup(addrSenders);
        rssi2 = advertisedDevice.getRSSI();
        freeFlag2 = false;
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      if(strcmp(addrSenders, sender3) == 0) {
        addrSender3 = strdup(addrSenders);
        rssi3 = advertisedDevice.getRSSI();
        freeFlag3 = false;
        Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      }
      free((char*)addrSenders); //free memory dat
    }
};


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
      cardidSticker = 0;
      timeLastCardRead = millis();
    }

    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    // readerDisabled = true;
} 

void Task1code( void * parameter) {
  
  for(;;) {
      // put your main code here, to run repeatedly:
    // BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    // Serial.print("Devices found: ");
    // Serial.println(foundDevices.getCount());
    // Serial.println("Scan done!");
    // pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    // delay(2000);
  }
}

void setup()
{
  //geluid:

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  audioLogger = &Serial;

  file = new AudioFileSourcePROGMEM(enigma_mod, sizeof(enigma_mod) );
  file2 = new AudioFileSourcePROGMEM(enigmacoin_mod, sizeof(enigmacoin_mod) );
  out = new AudioOutputI2S(0, 1); //Uncomment this line, comment the next one to use the internal DAC channel 1 (pin25) on ESP32
  // out = new AudioOutputI2S();
  out->SetGain(0); //max gain is 4
  mod = new AudioGeneratorMOD();
  mod->SetBufferSize(1024);
  mod->SetSampleRate(44100);
  mod->SetStereoSeparation(32);
  mod->begin(file, out);
  //einde geluid

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

  // OTA
  // ota.setHostname("esphond");  
  // ota.setPassword("esphond");
  // ota.begin();

  ledcSetup(0, 10000, 8); // set PWM frequency to 10kHz, resolution to 8 bits
  ledcAttachPin(speakerPin, 0); // attach PWM output to the speaker pin

  Serial.begin(115200); //Adapt the platformio.ini with correct monitor_speed

  

  // BLEDevice::init("");

  
  //NFC-->
  Serial.println("Begin NFC532 Scanning Software.");

  nfc.begin();



  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN532 board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();

  startListeningToNFC();
  //<--NFC
}

void loop()
{ 
  if (mod->isRunning()) {
    if (!mod->loop()) {
        mod->stop();
      }
  } else {
    Serial.print("MOD done");
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
    //kheb lik geen andere manier gevonden dan telkens nieuwe file aan te maken. je kan pointer lik wel op 0 zetten door seek functie, 
    //mo da werkt lik nie om 1 of andere reden dak nu nog nie heb gevonden. opzich is da zo erg nie, we gaan er wel op moeten letten 
    //dat file eerst ingeladen/gemaakt is met new en dan gain te bepalen(max 4) met afstand. zodat geluid nie achter komt
    // if(cardidSticker != 0) cardidSticker = 0;
  }

  // if((toneFrequency != 0) and (tellen < 1)){
  //   ledcWriteTone(0, toneFrequency);
  //   tellen = tellen +1;
  // }
  // else{
  //   ledcWrite(0,0);
  // } TODO alles met die tonefrequency, geen idee wat da is, maar da is precies ni nodig.


  if (readerDisabled) {
    if (millis() - timeLastCardRead > DELAY_BETWEEN_CARDS) {
      readerDisabled = false;
      startListeningToNFC();
    }
  } else {
    irqCurr = digitalRead(PN532_IRQ);
    // When the IRQ is pulled low - the reader has got something for us.
    if (irqCurr == LOW && irqPrev == HIGH) {
       //Serial.println("Got NFC IRQ");  
       handleCardDetected();
    }
    irqPrev = irqCurr;
  }





  //Eerst moet monster gedetecteerd worden, dan pas kan het beginnen zoeken naar de te zoeken drugs
  if(stickerDetected == true && cardidSticker == 1138283286187137){
    stickerDetected = false;
    monsterDetected = true; //TODO, opt einde vant spel moet da weer op false gezet worden, als alle zakjes gevonden zin dus wss.
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

  if(freeFlag1 == false){
    rssi1 = rssi1 + 30;
    rssi1 = (int) rssi1 / 4;
    Serial.print("sterkte signaal: ");
    Serial.println(rssi1);

    if (rssi1 == 0 || rssi1 == -1 || rssi1 == -2 || rssi1 == -3 || rssi1 == -4 || rssi1 == -5 || rssi1 == -6 || rssi1 == -7){
      gain = 3.5;//TODO: gain van 4 lukt lik niet, en kweet lik niet of een gain van 3.5 beter is dan 1. keer testen met de pcb en geluid file mss?
      //TODO: de gain moet ook weer op 0 gezet worden als er geen beacon gedetecteerd wordt.
    } else {
      gain = 0;
    }

    if(monsterDetected == true && strcmp(sender1,addrSender1) == 0){
      //TODO: hier moet ie blaffen
      Serial.println("stil BLAFFEN");
      Serial.println(gain);
      toneFrequency = 500;
      keuzeFile = 0; //0 = geblaf
    }
  }

  if(freeFlag2 == false && sticker2detected == true){
    rssi1 = rssi1 + 30;
    rssi1 = (int) rssi1 / 4;
    Serial.print("sterkte signaal: ");
    Serial.println(rssi1);

    if (rssi1 == 0 || rssi1 == -1 || rssi1 == -2 || rssi1 == -3 || rssi1 == -4 || rssi1 == -5 || rssi1 == -6 || rssi1 == -7){
      gain = 3.5;//TODO: gain van 4 lukt lik niet, en kweet lik niet of een gain van 3.5 beter is dan 1. keer testen met de pcb en geluid file mss?
      //TODO: de gain moet ook weer op 0 gezet worden als er geen beacon gedetecteerd wordt.
    } else {
      gain = 0;
    }

    if(monsterDetected == true && strcmp(sender2,addrSender2) == 0){
      //TODO: hier moet ie blaffen
      Serial.println("stil BLAFFEN");
      Serial.println(gain);
      toneFrequency = 500;
      keuzeFile = 0; //0 = geblaf
    }
  }

  if(freeFlag3 == false && sticker3detected == true && sticker2detected == true){
    rssi1 = rssi1 + 30;
    rssi1 = (int) rssi1 / 4;
    Serial.print("sterkte signaal: ");
    Serial.println(rssi1);

    if (rssi1 == 0 || rssi1 == -1 || rssi1 == -2 || rssi1 == -3 || rssi1 == -4 || rssi1 == -5 || rssi1 == -6 || rssi1 == -7){
      gain = 3.5;//TODO: gain van 4 lukt lik niet, en kweet lik niet of een gain van 3.5 beter is dan 1. keer testen met de pcb en geluid file mss?
      //TODO: de gain moet ook weer op 0 gezet worden als er geen beacon gedetecteerd wordt.
    } else {
      gain = 0;
    }

    if(monsterDetected == true && strcmp(sender1,addrSender1) == 0){
      //TODO: hier moet ie blaffen
      Serial.println("stil BLAFFEN");
      Serial.println(gain);
      toneFrequency = 500;
      keuzeFile = 0; //0 = geblaf
    }
  }

  if(cardidSticker == 1138283286187137 || cardidSticker == 1392551377325440 || cardidSticker == 1384850500963712 || cardidSticker == 1377149624601984){
    keuzeFile = 1;
    gain = 3.5; 
    
  }

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