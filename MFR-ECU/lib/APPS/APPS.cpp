#include "sensor.h"
#include <Arduino.h>

#define APPS1PIN 39
#define APPS2PIN 36

int AppsRead() {
    static uint8_t implausability = 0;
    int difference, Apps1deger, Apps2deger,Apps1,Apps2;
    float AppsOrtValue;
    
    Apps1deger= analogRead(APPS1PIN);
    Apps1 = Apps1deger*100/4095;
    Apps2deger= 2048-analogRead(APPS2PIN)*100/4095;         //  (Hangi pin olduğunu kontrol et)
    Apps2 = Apps2deger*100/4095;
    difference = abs(Apps1-Apps2);
    AppsOrtValue = ((Apps1deger+Apps2deger)/2) ;

    if (difference <= 10){
        return AppsOrtValue;
    } else {
        return -1;
    }
}