/*
    This is the combination of encoder class and vive class
    Note that the library ESP32Encoder (https://www.arduino.cc/reference/en/libraries/esp32encoder/) should be installed

*/

#ifndef S2_SENSORS_H
#define S2_SENSORS_H

#include <ESP32Encoder.h> // https://www.arduino.cc/reference/en/libraries/esp32encoder/
#include "s2_vive510.h"   // provided on canvas

class Sensors
{
public:
    // Encoder pins
    int ENCODER_L_A = 4;
    int ENCODER_L_B = 5;
    int ENCODER_R_A = 6;
    int ENCODER_R_B = 7;

    // Vive pin
    int VIVE_PIN_1 = 8;
    int VIVE_PIN_2 = 3;

    // Encoder objects
    ESP32Encoder encoder_L;
    ESP32Encoder encoder_R;

    // Vive object
    Vive510 vive1;
    Vive510 vive2;

    // Encoder values
    long encoder_L_val;
    long encoder_R_val;

    // Vive values
    int vive1_x;
    int vive1_y;
    int vive2_x;
    int vive2_y;

    // Constructor
    Sensors() : vive1(VIVE_PIN_1), vive2(VIVE_PIN_2),
                encoder_L_val(0), encoder_R_val(0),
                vive1_x(0), vive1_y(0), vive2_x(0), vive2_y(0)
    {
        encoder_L.attachFullQuad(ENCODER_L_A, ENCODER_L_B);
        encoder_R.attachFullQuad(ENCODER_R_A, ENCODER_R_B);
        vive1.begin();
        vive2.begin();
    }

    // copy constructor
    Sensors(const Sensors &old) = default;

    // destructor
    ~Sensors() = default;

    // update encoder values
    void updateEncoder()
    {
        encoder_L_val = encoder_L.getCount() / 2;
        encoder_R_val = encoder_R.getCount() / 2;
    }

    void updateVive()  // should be called in main loop before using vive values every time
    {
        if (vive1.status() == VIVE_RECEIVING)  // if vive receives signal
        {
            vive1_x = vive1.xCoord();
            vive1_y = vive1.yCoord();
        }
        else  // if vive does not receive signal, sync for 15 times, and don't update vive values
        {
            vive1.sync(15);
        }
        if (vive2.status() == VIVE_RECEIVING)
        {
            vive2_x = vive2.xCoord();
            vive2_y = vive2.yCoord();
        }
        else
        {
            vive2.sync(15);
        }
    }
};
#endif