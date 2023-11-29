/*
    Author:
        @jbwenjoy: Furina de Fontaine
    Description:
        This file defines the class for servo settings and control
*/

#ifndef SERVO_H
#define SERVO_H

// #include <Arduino.h>
// #include <driver/ledc.h>

class Servo
{
public:
    // Servo pin
    int SERVO_PWM;

    // LEDC channel
    int LEDC_CHANNEL;

    // PWM parameters
    int LEDC_RES_BITS;
    int LEDC_RES;
    int LEDC_FREQ;

    // Servo parameters
    int ANGLE_RANGE;  // 0 to ANGLE_RANGE degrees

    // Constructor
    Servo(int angle_range, int pwm_pin, int ledc_channel, int ledc_res_bits = 12, int ledc_freq = 50)
    {
        ANGLE_RANGE = angle_range;
        SERVO_PWM = pwm_pin;
        LEDC_CHANNEL = ledc_channel;
        LEDC_RES_BITS = ledc_res_bits;
        LEDC_RES = ((1 << LEDC_RES_BITS) - 1);
        LEDC_FREQ = ledc_freq;
    }

    // Copy constructor
    Servo(const Servo& old)
    {
        ANGLE_RANGE = old.ANGLE_RANGE;
        SERVO_PWM = old.SERVO_PWM;
        LEDC_CHANNEL = old.LEDC_CHANNEL;
        LEDC_RES_BITS = old.LEDC_RES_BITS;
        LEDC_RES = old.LEDC_RES;
        LEDC_FREQ = old.LEDC_FREQ;
    }

    // Destructor
    ~Servo() {}

    // Basic actions
    void setAngle(int angle)
    {
        // We would like mid angle to be 0 degree, and the servo can rotate ANGLE_RANGE degrees
        // angle is in range [-ANGLE_RANGE/2, ANGLE_RANGE/2]
        int duty = (LEDC_RES * (angle + ANGLE_RANGE / 2)) / ANGLE_RANGE;
        if (duty < 0)
        {
            duty = 0;
        }
        else if (duty > LEDC_RES)
        {
            duty = LEDC_RES;
        }
        ledcWrite(LEDC_CHANNEL, duty);
    }

};

#endif