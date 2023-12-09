/*
    Description: In this file, we define functions to communicate betweeen esp_c3 and esp_s2
*/

#ifndef C3_I2C_H
#define C3_I2C_H

#include <SoftwareSerial.h>
// Include any necessary headers here
class Serial_commun
{ // use this class in main to communicate between two boards. to initialize, give tx and rx
public:
    String message;
    SoftwareSerial softSerial;

    Serial_commun(int RX_PIN, int TX_PIN)
        : softSerial(RX_PIN, TX_PIN)
    {
        Serial.begin(115200);
        softSerial.begin(9600);
    }

    ~Serial_commun() {}

    void read()
    {
        if (softSerial.available())
        {
            message = softSerial.readString();
            Serial.println("Received message on ESP32 2: " + message);
        }
    }

    void write(String wr_message)
    {
        Serial.println("Sending message: " + wr_message);
        softSerial.println(wr_message);
    }
};

#endif