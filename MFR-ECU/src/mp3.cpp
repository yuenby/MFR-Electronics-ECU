#include <Arduino.h>

#ifdef ESP32
  #include <WiFi.h>
  #include "SPIFFS.h"
#else
  #include <ESP8266WiFi.h>
#endif
#include "AudioFileSourceSPIFFS.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

// To run, set your ESP8266 build to 160MHz, and include a SPIFFS of 512KB or greater.
// Use the "Tools->ESP8266/ESP32 Sketch Data Upload" menu to write the MP3 to SPIFFS
// Then upload the sketch normally.  

// pno_cs from https://ccrma.stanford.edu/~jos/pasp/Sound_Examples.html

AudioGeneratorMP3 *mp3;
AudioFileSourceSPIFFS *file1, *file2;
AudioOutputI2S *out;

void mp3_setup()
{
  WiFi.mode(WIFI_OFF); 
  Serial.begin(115200);
  delay(1000);
  SPIFFS.begin();
  Serial.printf("Sample MP3 playback begins...\n");

  out = new AudioOutputI2S(0,AudioOutputI2S::INTERNAL_DAC);
  out->SetGain(4.0);

  audioLogger = &Serial;
  file1 = new AudioFileSourceSPIFFS("/soundfile.mp3");
  file2 = new AudioFileSourceSPIFFS("/Error_siren.mp3");
  mp3 = new AudioGeneratorMP3();

  mp3->begin(file1, out);

}

void mp3_loop()
{
  if (mp3->isRunning()) {
    if (!mp3->loop()) mp3->stop();
  }
}

bool mp3_r2ds(){
  if (mp3->isRunning()) {
    return false;
  }else{
    mp3->begin(file1, out);
    return true;
  }
}

bool mp3_impausable(){
  if (mp3->isRunning()) {
    return false;
  }else{
    mp3->begin(file2, out);
    return true;
  }
}