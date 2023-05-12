// naam esp: espweegschaal
// wachtwoord esp: espweegschaal

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "OTAlib.h"
#include "HX711.h"
//change value's to your own wifi network and OTA settings
#define SSID "NETGEAR68"
#define PASSWORD "excitedtuba713"
#define OTAHOSTNAME "espweegschaal"
#define OTAPASSWORD "espweegschaal"

// Pins used for I2C IRQ
#define PN532_IRQ   4
#define PN532_RESET 5 
#define led 2
#define relais 32
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 17;
const int LOADCELL_SCK_PIN = 18;

bool card_detected[3] = {false,false,false};
bool detected[3] = {false,false,false};
bool readerDisabled = false;
const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
int irqCurr;
long zero;
long value;
int irqPrev;
int scalethreshold = 300000;

static void startListeningToNFC();
static void handleCardDetected();

OTAlib ota(SSID,PASSWORD);
WiFiClient espClient;
PubSubClient client(espClient);
HX711 scale;
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void blinkled(int amount){
  //used for visual feedback when board is used instead of serial monitor
  for(int i = 0; i < amount; i++){
    digitalWrite(led,HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    digitalWrite(led,LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void read_scale(int cardnr){
  if(card_detected[cardnr] == true and detected[cardnr] !=true){
    //reset scale to zero since a bag may be placed on it
    zero = scale.read();
    delay(100);
    value = 0;
    while(value-zero < scalethreshold){
      if (scale.wait_ready_timeout(100)) {
      value = scale.read();
      }
      Serial.println(value-zero);
      taskYIELD();
    }
    detected[cardnr] = true;
    blinkled(cardnr+1);
  }
}

void startListeningToNFC() {
  // Reset our IRQ indicators
  irqPrev = irqCurr = HIGH;
  nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
}

void handleCardDetected() {
    uint8_t success = false;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
    uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

    // read the NFC tag's info
    success = nfc.readDetectedPassiveTargetID(uid, &uidLength);
    if (success) {
      nfc.PrintHex(uid, uidLength);
      
      if(uidLength == 7){
        uint64_t cardid = uid[0];
        cardid <<= 8;
        cardid |= uid[1];
        cardid <<= 8;
        cardid |= uid[2];  
        cardid <<= 8;
        cardid |= uid[3]; 
        cardid <<= 8;
        cardid |= uid[4]; 
        cardid <<= 8;
        cardid |= uid[5]; 
        cardid <<= 8;
        cardid |= uid[6];
        Serial.print("Card ID NUMERIC Value: ");
        Serial.println(cardid);
        //set correct cardid to true
        if(cardid == 1210041037049217){
          card_detected[0] = true;
          blinkled(1);
        }
        if(cardid == 1201244944027009){
          card_detected[1] = true;
          blinkled(2);
        }
        if(cardid == 1192448851004801){
          card_detected[2] = true;
          blinkled(3);
        }
      }
      timeLastCardRead = millis();
    }
    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
}

void setup(void) {
  // OTA Setup
  ota.setHostname(OTAHOSTNAME);  
  ota.setPassword(OTAPASSWORD);
  ota.begin();
  //Serial Setup
  Serial.begin(115200);
  // RFID Setup
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    // Didn't find PN53x board
    while (1); // halt
  }
  nfc.SAMConfig();
  startListeningToNFC();
  //Scale Setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  //Relais Setup and test
  pinMode(32,OUTPUT);;
  digitalWrite(32,HIGH);

  pinMode(led,OUTPUT);

}

void loop(void) {  
  // delay for NFC reading
  //when not all cards are detected the code will run again
  if (!(detected[0] && detected[1] && detected[2])){
    if (readerDisabled) {
        if (millis() - timeLastCardRead > DELAY_BETWEEN_CARDS) {
          readerDisabled = false;
          startListeningToNFC();
        }
      } else {
        irqCurr = digitalRead(PN532_IRQ);
        // When the IRQ is pulled low - the reader has got something for us.
        if (irqCurr == LOW && irqPrev == HIGH) {
          handleCardDetected(); 
        }
      
        irqPrev = irqCurr;
      }
      //read scale values for each card if a card is detected
      read_scale(0);
      read_scale(1);
      read_scale(2);
  }
  else {
    //if all cards are detected the relais will be turned on
    digitalWrite(relais,LOW);
    //reset the bolean values to ensure the code doesn't start over again
  }
  taskYIELD();
}