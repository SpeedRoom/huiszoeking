// https://github.com/earlephilhower/ESP8266Audio

#include <Arduino.h>
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorMOD.h"
#include "AudioOutputI2S.h"
#include "OTAlib.h"

//OTA
// OTAlib ota("192.168.1.35", "excitedtuba713"); //TODO werkt nog nie, over the air

#if defined(ARDUINO_ARCH_RP2040)
    #define WIFI_OFF
    class __x { public: __x() {}; void mode() {}; };
    __x WiFi;
#elif defined(ESP32)
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif

// enigma.mod sample from the mod archive: https://modarchive.org/index.php?request=view_by_moduleid&query=42146
#include "enigma.h"

AudioGeneratorMOD *mod;
AudioFileSourcePROGMEM *file;
AudioFileSourcePROGMEM *file2;
AudioOutputI2S *out;

void setup()
{
  // ota.setHostname("esphond");  
  // ota.setPassword("esphond");
  // ota.begin();

  WiFi.mode(WIFI_OFF); //WiFi.forceSleepBegin();
  Serial.begin(115200);
  delay(1000);

  audioLogger = &Serial;
  file = new AudioFileSourcePROGMEM(enigma_mod, sizeof(enigma_mod) );
  file2 = new AudioFileSourcePROGMEM(enigmacoin_mod, sizeof(enigmacoin_mod) );
  out = new AudioOutputI2S(0, 1); //Uncomment this line, comment the next one to use the internal DAC channel 1 (pin25) on ESP32
  // out = new AudioOutputI2S();
  out->SetGain(4); //max gain is 4
  mod = new AudioGeneratorMOD();
  mod->SetBufferSize(3*1024);
  mod->SetSampleRate(44100);
  mod->SetStereoSeparation(32);
  mod->begin(file, out);
}

void loop()
{
  
  if (mod->isRunning()) {
    if (!mod->loop()) mod->stop(); //mod->stop() moet niet echt denk ik
  } else {
    Serial.printf("MOD done\n");
    delay(1000);
    file->close();
    delete file;
    file2 = new AudioFileSourcePROGMEM(enigmacoin_mod, sizeof(enigmacoin_mod));
    mod->begin(file2, out);
    //kheb lik geen andere manier gevonden dan telkens nieuwe file aan te maken. je kan pointer lik wel op 0 zetten door seek functie, 
    //mo da werkt lik nie om 1 of andere reden dak nu nog nie heb gevonden. opzich is da zo erg nie, we gaan er wel op moeten letten 
    //dat file eerst ingeladen/gemaakt is met new en dan gain te bepalen(max 4) met afstand. zodat geluid nie achter komt
  }
  Serial.printf("test");
}
