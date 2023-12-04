/*
    Authors: 

        @YuruiWang821: Yurui

        @jbwenjoy: Furina de Fontaine
    
    Description:

        In this file, we define functions for PID control.
        This class would be a template for both position and velocity control.
        The control functions need to accept the reference signal, the actual signal, and also PID parameters as arguments.
        PID global variables such as the error, integral, and derivative terms should be defined and stored in the PID class.
        So in the main ino file, we need to first define and initialize the PID class, then call the control functions in the loop.
        
        When implementing this class, you should also include the actions.h file, 
        as you will need to read and modify the global variables defined in actions.h.
        See actions.h for more details.
*/

#pragma once
#ifndef CONTROL_H
#define CONTROL_H

class PIDController  // Note that all variables here are float
{
private:
    // PID intermediate variables
    float error;
    float error_previous;
    float integral;
    float derivative;
    float time_current_ms;
    float time_previous_ms;

    float PIDSaturation(float PID_output, float upper_bound = 4095, float lower_bound = -4095)
    {
        // Saturation
        if (PID_output > upper_bound)
        {
            PID_output = upper_bound;
        }
        else if (PID_output < lower_bound)
        {
            PID_output = lower_bound;
        }

        return PID_output;
    }

public:
    // PID parameters
    float kp;
    float ki;
    float kd;
    
    // PID output
    float output;
    
    // Constructor, default parameters are: kp=2.0, ki=0.3, kd=0.001
    PIDController(float kp=15.0, float ki=0.05, float kd=0.05)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        time_current_ms = (float) millis();
        time_previous_ms = (float) millis();
    }

    // Destructor
    ~PIDController() {}

    // Template PID control function
    float PID(float reference, float actual)
    {
        time_current_ms = (float) millis();
        float time_delta_ms = time_current_ms - time_previous_ms;
        // Serial.print(time_delta_ms);

        // Calculate error
        error = reference - actual;

        // Calculate integral
        integral += error * time_delta_ms;

        // Intergral anti-saturation
        // if (error * error_previous < 0) integral = 0;
        if (integral > 20000) integral = 20000;
        if (integral < -20000) integral = -20000;

        // // Calculate integral, and only update integral value when error is small
        // if (abs(error) < 100) integral += error;

        // Calculate derivative
        derivative = (error - error_previous) / time_delta_ms;
        
        // Calculate output
        // output = kp * error + ki * integral + kd * derivative;
        output = kp * error + ki * integral + constrain(kd * derivative, -0.1 * kp * error, 0.1 * kp * error);

        // Update previous
        error_previous = error;
        time_previous_ms = time_current_ms;

        output = PIDSaturation(output);

        // Serial.print('\t');
        // Serial.print(integral);

        return output;
    }

    // Control functions
    // float positionControl(float reference, float actual)
    // {
    //     // TODO: Implement position control
    //     ;
    // }


    // float velocityControl(float reference, float actual)
    // {
    //     // TODO: Implement velocity control
    //     ;
    // }


};

#endif
