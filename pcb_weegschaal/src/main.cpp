// naam esp: esphond
// wachtwoord esp: esphond

#include <Arduino.h>
#include "sys/time.h"
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
#include "OTAlib.h"

//OTA
OTAlib ota("NETGEAR68", "excitedtuba713");

static void startListeningToNFC();
static void handleCardDetected();

// Pins used for I2C IRQ
#define PN532_IRQ   4
#define PN532_RESET 5 

int countDevices = 0;
int sum_rssi = 0;
int rssi;
const int DELAY_BETWEEN_CARDS = 500;
long timeLastCardRead = 0;
boolean readerDisabled = false;
int irqCurr;
int irqPrev;

WiFiClient espClient;
PubSubClient client(espClient);

uint64_t cardidSticker;
bool stickerDetected = false;
bool monsterDetected = false;
const char * addrSender;
bool freeFlag = true;


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

class IBeaconAdvertised : public BLEAdvertisedDeviceCallbacks
{
public:
  // Callback when BLE is detected
  void onResult(BLEAdvertisedDevice device)
  {
    if (!isIBeacon(device))
    {
      return;
    }
    printIBeacon(device);
    countDevices +=1;
  }

private:
  // iBeacon packet judgment
  bool isIBeacon(BLEAdvertisedDevice device)
  {
    if (device.getManufacturerData().length() < 25)
    {
      return false;
    }
    if (getCompanyId(device) != 0x004C)
    {
      return false;
    }
    if (getIBeaconHeader(device) != 0x1502)
    {
      return false;
    }
    return true;
  }

  // CompanyId acquisition
  unsigned short getCompanyId(BLEAdvertisedDevice device)
  {
    const unsigned short *pCompanyId = (const unsigned short *)&device
                                           .getManufacturerData()
                                           .c_str()[0];
    return *pCompanyId;
  }

  // Get iBeacon Header
  unsigned short getIBeaconHeader(BLEAdvertisedDevice device)
  {
    const unsigned short *pHeader = (const unsigned short *)&device
                                        .getManufacturerData()
                                        .c_str()[2];
    return *pHeader;
  }

  // Get UUID
  String getUuid(BLEAdvertisedDevice device)
  {
    const char *pUuid = &device.getManufacturerData().c_str()[4];
    char uuid[64] = {0};
    sprintf(
        uuid,
        "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
        pUuid[0], pUuid[1], pUuid[2], pUuid[3], pUuid[4], pUuid[5], pUuid[6], pUuid[7],
        pUuid[8], pUuid[9], pUuid[10], pUuid[11], pUuid[12], pUuid[13], pUuid[14], pUuid[15]);
    return String(uuid);
  }

  // Get TxPower
  signed char getTxPower(BLEAdvertisedDevice device)
  {
    const signed char *pTxPower = (const signed char *)&device
                                      .getManufacturerData()
                                      .c_str()[24];
    return *pTxPower;
  }

  // Serial output of iBeacon information
  void printIBeacon(BLEAdvertisedDevice device)
  {
    if (! (getUuid(device).equals("2686F39C-BADA-4658-854A-A62E7E5E8B8") || getUuid(device).equals("2686F39C-BADA-4658-854A-A62E7E5E8B8D"))){
      addrSender =  strdup(device.getAddress().toString().c_str());
      freeFlag = false;
      Serial.println("");
      Serial.printf("addr:%s rssi:%d uuid:%s power:%d\r\n",
                    device.getAddress().toString().c_str(), //BELANGRIJK! dit is het adres afhankelijk van verzender!
                    device.getRSSI(),
                    getUuid(device).c_str(),
                    *(signed char *)&device.getManufacturerData().c_str()[24]);
      Serial.println("");
      sum_rssi += device.getRSSI();
    }
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
      success = false;
      

      timeLastCardRead = millis();
    }

    // The reader will be enabled again after DELAY_BETWEEN_CARDS ms will pass.
    readerDisabled = true;
}

void setup()
{
  // OTA
  ota.setHostname("esphond");  
  ota.setPassword("esphond");
  ota.begin();

  ledcSetup(0, 10000, 8); // set PWM frequency to 10kHz, resolution to 8 bits
  ledcAttachPin(speakerPin, 0); // attach PWM output to the speaker pin

  Serial.begin(115200); //Adapt the platformio.ini with correct monitor_speed

  BLEDevice::init("");

  
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

  if((toneFrequency != 0) and (tellen < 1)){
    ledcWriteTone(0, toneFrequency);
    tellen = tellen +1;
  }
  else{
    ledcWrite(0,0);
  }

  //ontvangen signaal
  
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new IBeaconAdvertised(), true);
  scan->setActiveScan(true);
  scan->start(1, false);

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
  
  if(stickerDetected == true && cardidSticker == 1138283286187137){
    stickerDetected = false;
    monsterDetected = true; //TODO, opt einde vant spel moet da weer op false gezet worden, als alle zakjes gevonden zin dus wss.
    Serial.println("heeft gesnuffeld");
    cardidSticker = 0;
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1392551377325440){
    //hier sticker 2 gedetecteerd // TODO: hier moet ie dus nog luider/wilder blaffen
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    toneFrequency = 3000;
    tellen = 0;
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1384850500963712){
    //hier sticker 3 gedetecteerd // TODO: hier moet ie dus nog luider blaffen
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    toneFrequency = 3000;
    tellen = 0;
  }

  if(stickerDetected == true && monsterDetected == true && cardidSticker == 1377149624601984){
    //hier sticker 4 gedetecteerd // TODO: hier moet ie dus nog luider blaffen
    stickerDetected = false;
    Serial.println("luid BLAFFEN");
    toneFrequency = 3000;
    tellen = 0;
  }
  const char * x = "7c:9e:bd:2a:fc:1e";//TODO: als deze gevonden is, adres veranderen naar volgend drugszakje
  if(freeFlag == false && countDevices >= 5){ //countdevices moet aan 5 zijn om gemiddelde daaruit te berekenen
    rssi = (int) sum_rssi / countDevices;
    rssi = rssi + 30;
    rssi = (int) rssi / 4;
    countDevices = 0;
    sum_rssi = 0;
    Serial.printf(addrSender);
    Serial.println("");
    Serial.println(rssi);
    // if(monsterDetected == true && (strcmp(addrSender,x) == 0 || strcmp(addrSender,x) == 0 || strcmp(addrSender,x) == 0)){
    if(monsterDetected == true && strcmp(x,addrSender) == 0 && rssi > -65){
      //TODO: hier moet ie blaffen
      Serial.println("stil BLAFFEN");
      toneFrequency = 500;
      tellen = 0;
    }
  }
  Serial.print("...");

  if(freeFlag == false){
    free((char*)addrSender);
    freeFlag = true;
  }
}