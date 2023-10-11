#include <Arduino.h>
#include <NEXTlcd.h>

nextlcd::nextlcd(HardwareSerial *serial = &Serial2){
    _serial = serial;
}

void nextlcd::writeSensor(int rpm, float currentVal){

    int angle_rpm = (rpm * 3 / 100 + 330) % 360;
    int angle_current = (int(currentVal) *3 + 330) % 360;

    _serial->print("surus.textRPMValue.txt=\"");
    _serial->print(rpm);
    _serial->print("\"\xFF\xFF\xFF");
    _serial->print("surus.textCurrValue.txt=\"");
    _serial->print(currentVal);
    _serial->print("\"\xFF\xFF\xFF");
    _serial->print("surus.gaugeRPM.val=");
    _serial->print(angle_rpm);
    _serial->print("\xFF\xFF\xFF");
    _serial->print("surus.gaugeCurrent.val=");
    _serial->print(angle_current);
    _serial->print("\xFF\xFF\xFF");
}

void nextlcd::changeMode(int mod){
    if(mod == 1) { //mod 1 = surus modu;
        _serial->print("page surus");
    }
    else if(mod == 2){ // mod 2 = otonom;
        _serial->print("page otonom");
    }
    Serial.print("\xFF\xFF\xFF");
}

void nextlcd::changeStatus(String status){
    _serial->print("otonom.textStatus.txt=\"");
    _serial->print(status);
    _serial->print("\"\xFF\xFF\xFF");
}