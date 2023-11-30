/*
    This is the combination of encoder class and vive class
    Note that the library ESP32Encoder (https://www.arduino.cc/reference/en/libraries/esp32encoder/) should be installed
    
*/

#ifndef S2_SENSORS_H
#define S2_SENSORS_H

#include "ESP32Encoder.h"  // https://www.arduino.cc/reference/en/libraries/esp32encoder/
#include "s2_vive510.h"  // provided on canvas

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
    long encoder_L_val = 0;
    long encoder_R_val = 0;

    // Vive values
    int vive1_x = 0;
    int vive1_y = 0;
    int vive2_x = 0;
    int vive2_y = 0;

    // Constructor
    Sensors() : vive1(VIVE_PIN_1), vive2(VIVE_PIN_2)
    {
        encoder_L.attachFullQuad(ENCODER_L_A, ENCODER_L_B);
        encoder_R.attachFullQuad(ENCODER_R_A, ENCODER_R_B);
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

};
#endif