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


//last ting i was doing
//testing if the code can be shorter see bag nr2
//test if the code fully works?

//change value's to your own wifi network
#define SSID "NETGEAR68"
#define PASSWORD "excitedtuba713"

// Pins used for I2C IRQ
#define PN532_IRQ   4
#define PN532_RESET 5 
#define led 2
#define relais 32
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 17;
const int LOADCELL_SCK_PIN = 18;

bool card1_detected = false;
bool card2_detected = false;
bool card3_detected = false;
bool detected1 = false;
bool detected2 = false;
bool detected3 = false;
long zero;
long value;
const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
boolean readerDisabled = false;
int irqCurr;
int irqPrev;
int scalethreshold = 300000;

static void startListeningToNFC();
static void handleCardDetected();

OTAlib ota(SSID,PASSWORD);

WiFiClient espClient;
PubSubClient client(espClient);

HX711 scale;


// This example uses the IRQ line, which is available when in I2C mode.
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void setup(void) {
  // OTA Setup
  ota.setHostname("espweegschaal");  
  ota.setPassword("espweegschaal");
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
  
  pinMode(led,OUTPUT);
  //Relais Setup and test
  pinMode(32,OUTPUT);;
  digitalWrite(32,HIGH);
}

void blinkled(){
  digitalWrite(led,HIGH);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(led,LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
}

void loop(void) {  
  // delay for NFC reading
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
  if(card1_detected == true and detected1 !=true){
    //reset scale to zero since a bag is placed on it
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
    detected1 = true;
    blinkled();
    
  }

  if(card2_detected == true and detected2 !=true){
    //reset scale to zero since a bag is placed on it
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
    detected2 = true;
    blinkled();
  }
  
  if(card3_detected == true and detected3 !=true){
    //reset scale to zero since a bag is placed on it
    zero = scale.read();
    delay(100);
    value = 0;
    Serial.println(value-zero);
    while(value-zero < scalethreshold){
      if (scale.wait_ready_timeout(100)) {
      value = scale.read();
      }
      Serial.println(value-zero);
      taskYIELD();
    }
    detected3 = true;
    blinkled();
  }
  if (detected1 && detected2 && detected3){
    digitalWrite(relais,LOW);
    detected1 = false;
    detected2 = false;
    detected3 = false;
    card1_detected = false;
    card2_detected = false;
    card3_detected = false;
    //reset the bolean values to ensure the code doesn't start over again
  }
  taskYIELD();


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
          card1_detected = true;
        }
        if(cardid == 1201244944027009){
          card2_detected = true;
        }
        if(cardid == 1192448851004801){
          card3_detected = true;
        }
      }
      // Wait a bit before scanning again
      timeLastCardRead = millis();
    }

    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
}