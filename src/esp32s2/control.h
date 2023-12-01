/*
    Authors: 

        @YuruiWang821: Yurui
    
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

class PIDController
{
private:
    // PID variables
    float error;
    float error_previous;
    float integral;
    float derivative;

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

        // Calculate integral
        integral += error;

        // Calculate derivative
        derivative = error - error_previous;

        // Calculate output
        output = kp * error + ki * integral + kd * derivative;

        // Update error_previous
        error_previous = error;

        return output;
    }

    // Control functions
    float positionControl(float reference, float actual)
    {

    }


    float velocityControl(float reference, float actual)
    {

    }


};

#endif
