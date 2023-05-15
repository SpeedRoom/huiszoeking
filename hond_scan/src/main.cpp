#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>

int rssi1;
int rssi2;
int rssi3;

const char * addrSenders;
const char * addrSender1;
const char * addrSender2;
const char * addrSender3;
bool freeFlag = true;
bool freeFlag1 = true;
bool freeFlag2 = true;
bool freeFlag3 = true;
const char * sender1 = "7c:9e:bd:ed:58:1a";//TODO: als deze gevonden is, adres veranderen naar volgend drugszakje 7c:9e:bd:ed:58:1a 
const char * sender2 = "7c:9e:bd:ed:58:1a";
const char * sender3 = "7c:9e:bd:ed:58:1a";

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

void setup() {
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

}

void loop() {
}