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
    int ENCODER_L_A = 7;
    int ENCODER_L_B = 4;

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
    int ONE_ROUND = 988;                                       // encoder value when the wheel goes 1 round
    float WHEEL_PERIMETER = 205;                               // mm
    float DIST_PER_COUNT = WHEEL_PERIMETER / (float)ONE_ROUND; // 0.2075 mm/count
    long encoder_L_val;
    long encoder_R_val;
    long encoder_L_val_previous;
    long encoder_R_val_previous;
    unsigned long encoder_time_current;  // ms
    unsigned long encoder_time_previous; // ms
    float speed_L;                       // mm/s
    float speed_R;                       // mm/s
    float count_speed_L;                 // counts per ms
    float count_speed_R;                 // counts per ms

    // Vive values
    int vive1_x;
    float vive1_x_mm;
    int vive1_y;
    float vive1_y_mm;
    int vive2_x;
    float vive2_x_mm;
    int vive2_y;
    float vive2_y_mm;

    // position calculated from vive values
    float vive_x_mm;
    float vive_y_mm;
    // orientation calculated from vive values
    float vive_theta; // in degrees

    // Constructor
    Sensors() : vive1(VIVE_PIN_1), vive2(VIVE_PIN_2),
                encoder_L_val(0), encoder_R_val(0),
                encoder_L_val_previous(0), encoder_R_val_previous(0),
                speed_L(0.0), speed_R(0.0),
                count_speed_L(0.0), count_speed_R(0.0),
                vive1_x(0), vive1_y(0), vive2_x(0), vive2_y(0),
                vive_x_mm(0.0), vive_y_mm(0.0), vive_theta(0.0)
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
        speed_L = count_speed_L * DIST_PER_COUNT * 1000;
        speed_R = count_speed_R * DIST_PER_COUNT * 1000;
    }

    void updateVive() // should be called in main loop before using vive values every time
    {
        // Serial.print(vive1.status());
        // Serial.print('\t');
        if (vive1.status() == VIVE_RECEIVING) // if vive receives signal
        {
            vive1_x = vive1.xCoord();
            vive1_y = vive1.yCoord();
        }
        else // if vive does not receive signal, sync for 15 times, and don't update vive values
        {
            vive1.sync(15);
        }
        // Serial.print(vive2.status());
        // Serial.print('\n');
        if (vive2.status() == VIVE_RECEIVING)
        {
            vive2_x = vive2.xCoord();
            vive2_y = vive2.yCoord();
        }
        else
        {
            vive2.sync(15);
        }
        
        vive1_x_mm = calculateXCoordInMillimeters(vive1_x, vive1_y);
        vive1_y_mm = calculateYCoordInMillimeters(vive1_x, vive1_y);
        vive2_x_mm = calculateXCoordInMillimeters(vive2_x, vive2_y);
        vive2_y_mm = calculateYCoordInMillimeters(vive2_x, vive2_y);
        
        calculateVivePosition();  // update vive_x_mm and vive_y_mm
        calculateViveOrientation();  // update vive_theta
    }

    void calculateViveOrientation()
    {
        // TODO: calculate vive orientation
        // Here we assume that the two vive sensors are symmetricly located on the left and right side of the robot
        // So when we select x+ axis as 0 degree, the heading of the robot is
        auto dx = vive2_x_mm - vive1_x_mm;
        auto dy = vive2_y_mm - vive1_y_mm;
        vive_theta = atan2(dy, dx) * 180 / PI + 90;  // +90 or -90 depends on the vive sensor location
        // Convert to -180 ~ 180
        if (vive_theta > 180)
        {
            vive_theta -= 360;
        }
        else if (vive_theta < -180)
        {
            vive_theta += 360;
        }

    }

    void calculateVivePosition()
    {
        // Calculate vive position
        // Here we assume that the two vive sensors are symmetricly located on the left and right side of the robot
        // So the center of the robot is the midpoint of the two vive sensors
        vive_x_mm = (vive1_x_mm + vive2_x_mm) / 2;
        vive_y_mm = (vive1_y_mm + vive2_y_mm) / 2;
    }

    float calculateXCoordInMillimeters(int vive_coord_x, int vive_coord_y)
    {
        // Compute the actual position in millimeters
        int x_min = 1500;
        int delta_x = 4400;
        int L_x = 3556; // mm

        float x_mm = (float) ((vive_coord_x - x_min) * L_x) / (float) delta_x;
        return x_mm;
    }

    float calculateYCoordInMillimeters(int vive_coord_x, int vive_coord_y)
    {
        // Compute the actual position in millimeters
        int x_min = 1500;
        int y_min_1 = 2700;
        int y_min_2 = 2900;
        int delta_x = 4400;
        int delta_y = 2200;
        int L_x = 3556; // mm
        int L_y = 1778; // mm

        int y_min_ = (vive_coord_x - x_min) * (y_min_2 - y_min_1) / delta_x + y_min_1;
        float y_mm = (float) ((vive_coord_y - y_min_) * L_y) / (float) delta_y;
        return y_mm;
    }

    bool atDesiredOrientation(float desired_theta, float threshold = 5.0) // alsolute angle from vive, currently unable to deal with overshot
    {
        calculateViveOrientation();
        // float threshold = 5.0; // degree, to be tuned
        return (abs(vive_theta - desired_theta) <= threshold);
    }

    bool atDesiredPosition(float desired_x, float desired_y, float threshold = 10.0) // alsolute position from vive, currently unable to deal with overshot
    {
        calculateVivePosition();
        // float threshold = 10.0; // mm, to be tuned
        return (abs(vive_x_mm - desired_x) <= threshold) && (abs(vive_y_mm - desired_y) <= threshold);
    }

};

#endif
