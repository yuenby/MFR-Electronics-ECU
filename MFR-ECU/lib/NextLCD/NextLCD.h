#ifndef NextLCD_h
#define NextLCD_h

class nextlcd
{
public:
    nextlcd(HardwareSerial *serial);
    void writeSensor(int rpm, float currentVal); // Now we both override and overload the function write from the base class
    void changeMode(int mod);
    void changeStatus(String status);
private:
    HardwareSerial *_serial;
};

#endif