/*
    Author: 
        @jbwenjoy: Furina de Fontaine
    Description:
        This file defines functions for motor control.
*/

#ifndef MOTOR_H
#define MOTOR_H

// #include <Arduino.h>
// #include <driver/ledc.h>

class Motor
{
public:
    // Motor pins
    int MOTOR_PWM;
    int MOTOR_DIR1;
    int MOTOR_DIR2;

    // LEDC channels
    int LEDC_CHANNEL;   

    // PWM parameters
    int LEDC_RES_BITS;
    int LEDC_RES;
    int LEDC_FREQ;

    // Constructor
    Motor(int pwm_pin, int dir1_pin, int dir2_pin, int ledc_channel, int ledc_res_bits = 12, int ledc_freq = 5000)
    {
        MOTOR_PWM = pwm_pin;
        MOTOR_DIR1 = dir1_pin;
        MOTOR_DIR2 = dir2_pin;
        LEDC_CHANNEL = ledc_channel;
        LEDC_RES_BITS = ledc_res_bits;
        LEDC_RES = ((1 << LEDC_RES_BITS) - 1);
        LEDC_FREQ = ledc_freq;
    }

    // Copy constructor
    Motor(const Motor& old)
    {
        MOTOR_PWM = old.MOTOR_PWM;
        MOTOR_DIR1 = old.MOTOR_DIR1;
        MOTOR_DIR2 = old.MOTOR_DIR2;
        LEDC_CHANNEL = old.LEDC_CHANNEL;
        LEDC_RES_BITS = old.LEDC_RES_BITS;
        LEDC_RES = old.LEDC_RES;
        LEDC_FREQ = old.LEDC_FREQ;
    }

    // Destructor
    ~Motor() {}

    // Basic actions
    void setSpeed(int speed)
    {
        // speed is in range [-4095, 4095]
        if (speed > 0)
        {
            digitalWrite(MOTOR_DIR1, HIGH);
            digitalWrite(MOTOR_DIR2, LOW);
        }
        else if (speed < 0)
        {
            digitalWrite(MOTOR_DIR1, LOW);
            digitalWrite(MOTOR_DIR2, HIGH);
        }
        else
        {
            digitalWrite(MOTOR_DIR1, LOW);
            digitalWrite(MOTOR_DIR2, LOW);
        }
        ledcWrite(LEDC_CHANNEL, abs(speed));
    }
};


#endif