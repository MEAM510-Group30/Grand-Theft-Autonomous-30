/*
    Authors: 
    
        @jbwenjoy: Furina de Fontaine
    
    Description:
    
        This is the combination of encoder class and vive class
        Note that the library ESP32Encoder v0.8.0 (https://www.arduino.cc/reference/en/libraries/esp32encoder/) should be installed, higher version may not work
*/

#ifndef S2_SENSORS_H
#define S2_SENSORS_H

#include <ESP32Encoder.h> // https://www.arduino.cc/reference/en/libraries/esp32encoder/ v0.8.0
#include "s2_vive510.h"   // provided on canvas

class Sensors
{
public:
    // Encoder pins, have been checked to match our hardware
    int ENCODER_R_A = 5;
    int ENCODER_R_B = 6;
    int ENCODER_L_A = 4;
    int ENCODER_L_B = 7;

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
    int ONE_ROUND = 988;  // encoder value when the wheel goes 1 round
    int WHEEL_PERIMETER = 205;  // mm
    int DIST_PER_COUNT = WHEEL_PERIMETER / ONE_ROUND;  // mm/count
    long encoder_L_val;
    long encoder_R_val;
    long encoder_L_val_previous;
    long encoder_R_val_previous;
    unsigned long encoder_time_current;  // ms
    unsigned long encoder_time_previous;  // ms
    float speed_L;  // mm/s
    float speed_R;  // mm/s
    float count_speed_L;  // encoder value per ms
    float count_speed_R;  // encoder value per ms

    // Vive values
    int vive1_x;
    int vive1_y;
    int vive2_x;
    int vive2_y;

    // Constructor
    Sensors() : vive1(VIVE_PIN_1), vive2(VIVE_PIN_2),
                encoder_L_val(0), encoder_R_val(0), 
                encoder_L_val_previous(0), encoder_R_val_previous(0),
                speed_L(0.0), speed_R(0.0),
                count_speed_L(0.0), count_speed_R(0.0),
                vive1_x(0), vive1_y(0), vive2_x(0), vive2_y(0)
    {
        ESP32Encoder::useInternalWeakPullResistors = UP;
        encoder_L.attachFullQuad(ENCODER_L_A, ENCODER_L_B);
        encoder_R.attachFullQuad(ENCODER_R_A, ENCODER_R_B);
        encoder_time_current = millis();
        encoder_time_previous = encoder_time_current;
        vive1.begin();
        vive2.begin();
    }

    // copy constructor
    Sensors(const Sensors &old) = default;

    // destructor
    ~Sensors() = default;

    // update encoder values, when using this library, getCount() increases by appr 988 when the wheels go 1 round
    void updateEncoder()
    {
        encoder_time_previous = encoder_time_current;
        encoder_L_val_previous = encoder_L_val;
        encoder_R_val_previous = encoder_R_val;
        encoder_time_current = millis();
        encoder_L_val = encoder_L.getCount() / 2;
        encoder_R_val = encoder_R.getCount() / 2;
        count_speed_L = (float)(encoder_L_val - encoder_L_val_previous) / (float)(encoder_time_current - encoder_time_previous);
        count_speed_R = (float)(encoder_R_val - encoder_R_val_previous) / (float)(encoder_time_current - encoder_time_previous);
        speed_L = count_speed_L * DIST_PER_COUNT;
        speed_R = count_speed_R * DIST_PER_COUNT;
    }

    void updateVive()  // should be called in main loop before using vive values every time
    {
        Serial.print(vive1.status());
        Serial.print('\t');
        if (vive1.status() == VIVE_RECEIVING)  // if vive receives signal
        {
            vive1_x = vive1.xCoord();
            vive1_y = vive1.yCoord();
        }
        else  // if vive does not receive signal, sync for 15 times, and don't update vive values
        {
            vive1.sync(15);
        }
        Serial.print(vive2.status());
        Serial.print('\n');
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