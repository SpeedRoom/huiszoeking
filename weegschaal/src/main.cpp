/**************************************************************************/
/*! 

This example is for communicating with the PN532 chip using I2C. Wiring 
should be as follows:
  PN532 SDA -> EFM32 D21 Pin (with 4.7K resistor already present)
  PN532 SCL -> EFM32 D22 Pin(with 4.7K resistor already present)
  PN532 IRQ -> EFM32 D4 Pin
  PN32 RST0 -> EFM32 D5 Pin
  PN532 3.3v -> 3.3v
  PN532 GND -> GND

Based on readMifareClassicIrq.pde by Adafruit
*/
/**************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include "WiFi.h"
#include "PubSubClient.h"

#include "HX711.h"



static void startListeningToNFC();
static void handleCardDetected();

bool card1 = false;
bool card2 = false;
bool card3 = false;
bool detected1 = false;
bool detected2 = false;
bool detected3 = false;

long zero;
long value;


// Pins used for I2C IRQ
#define PN532_IRQ   4
#define PN532_RESET 5 
#define led 2

const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
boolean readerDisabled = false;
int irqCurr;
int irqPrev;

WiFiClient espClient;
PubSubClient client(espClient);
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 17;
const int LOADCELL_SCK_PIN = 16;

HX711 scale;

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

void setup(void) {
  pinMode(led, OUTPUT);
  digitalWrite(led,LOW);
  Serial.begin(115200); //Adapt the platformio.ini with correct monitor_speed

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

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop(void) {  
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

  if(card1 == true and detected1 !=true){
    Serial.println("foudn nr1");
    if (scale.wait_ready_timeout(100)) {
    zero = scale.read();
    Serial.println("zero gemeten");
    delay(100);
    }
    if (scale.wait_ready_timeout(100)) {
    value = scale.read();
    Serial.println(value);
    }
    Serial.println(value-zero);
    while(value-zero < 3000){
      if (scale.wait_ready_timeout(100)) {
      value = scale.read();
      }
      Serial.println(value-zero);
    }
    Serial.println("joepie zakje nr 1 licht in de bak");
    detected1 = true;
    card1 = false;
  }

  if(card2 == true and detected2 !=true){
    Serial.println("foudn nr2");
    if (scale.wait_ready_timeout(100)) {
    zero = scale.read();
    Serial.println("zero gemeten");
    delay(100);
    }
    if (scale.wait_ready_timeout(100)) {
    value = scale.read();
    Serial.println(value);
    }
    Serial.println(value-zero);
    while(value-zero < 3000){
      if (scale.wait_ready_timeout(100)) {
      value = scale.read();
      }
      Serial.println(value-zero);
    }
    Serial.println("joepie zakje nr 1 licht in de bak");
    detected2 = true;
    card2 = false;
  }
  
  if(card3 == true and detected3 !=true){
    if (scale.wait_ready_timeout(100)) {
    zero = scale.read();
    Serial.println("zero gemeten");
    delay(100);
    }
    if (scale.wait_ready_timeout(100)) {
    value = scale.read();
    Serial.println(value);
    }
    Serial.println(value-zero);
    while(value-zero < 3000){
      if (scale.wait_ready_timeout(100)) {
      value = scale.read();
      }
      Serial.println(value-zero);
    }
    Serial.println("joepie zakje nr 1 licht in de bak");
    detected3 = true;
    card3 = false;
  }



}

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
        if(cardid == 1157230119177600){
          card1 = true;
        }
        if(cardid == 1139637933133184){
          card2 = true;
        }
        if(cardid == 1141832661421440){
          card3 = true;
        }
      }
      



      if (uidLength == 4)
      {
        // We probably have a Mifare Classic card ... 
        uint32_t cardid = uid[0];
        cardid <<= 8;
        cardid |= uid[1];
        cardid <<= 8;
        cardid |= uid[2];  
        cardid <<= 8;
        cardid |= uid[3]; 
        //Serial.print("Seems to be a Mifare Classic card #");
        Serial.print("Card ID NUMERIC Value: ");
        Serial.println(cardid);
        const char * mes = (const char *) String(cardid).c_str();
        Serial.println(mes);
        client.publish("nieuwpoort/tags", mes);
      }
      Serial.println("");

      

      timeLastCardRead = millis();
    }

    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
}
/*
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop() {

  if (scale.wait_ready_timeout(100)) {
    long reading = scale.read();
    Serial.print("HX711 reading: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not found.");
  }

  delay(150);
  
}
*/