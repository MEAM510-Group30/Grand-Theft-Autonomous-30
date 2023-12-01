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
    
    // Constructor
    PIDController(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    // Destructor
    ~PIDController() {}

    // Template PID control function
    float PID(float reference, float actual)
    {
        // Calculate error
        error = reference - actual;

        // Intergral reset when error changes sign
        if (error * error_previous < 0)
        {
            integral = 0;
        }

        // Calculate integral
        integral += error;

        // Calculate derivative
        derivative = error - error_previous;

        // Calculate output
        // output = kp * error + ki * integral + kd * derivative;
        output = kp * error + ki * integral + constrain(kd * derivative, -0.1 * kp * error, 0.1 * kp * error);

        // Update error_previous
        error_previous = error;

        output = PIDSaturation(output);

        return output;
    }

    // Control functions
    float positionControl(float reference, float actual)
    {
        // TODO: Implement position control
        ;
    }


    float velocityControl(float reference, float actual)
    {
        // TODO: Implement velocity control
        ;
    }


};

#endif
