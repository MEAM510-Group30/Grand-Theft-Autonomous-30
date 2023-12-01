/*
    Authors: 
        
        @jbwenjoy: Furina de Fontaine
    
    Description:

        This file defines functions for all kinds of actions.

        Two layers of actions should be defined here:
            1. Basic actions, like specifying motor speed, servo angle, etc.
            2. Upper-layer actions, like moving, turning, etc, which are combinations of basic actions.

*/

#ifndef ACTIONS_H
#define ACTIONS_H

#include "actions_motor.h"
#include "actions_servo.h"


enum MoveActionMode
{
    STOP,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

enum JawActionMode
{
    JAW_HOLD,  // keep the jaw at the current position
    JAW_GRAB,  // grab the trophy
    JAW_RELEASE  // release the trophy
};


class Actions  // all variables and functions are public
{
public:
    // Motor pins
    int MOTOR_L_PWM = 1;
    int MOTOR_R_PWM = 2;
    int MOTOR_L_DIR1 = 38;
    int MOTOR_L_DIR2 = 35;
    int MOTOR_R_DIR1 = 36;
    int MOTOR_R_DIR2 = 37;

    // Servo pins
    int SERVO_JAW_PWM = 13;
    int SERVO_IR_PWM = 14;

    // LEDC channels
    int LEDC_CHA_0 = 0;
    int LEDC_CHA_1 = 1;
    int LEDC_CHA_2 = 2;
    int LEDC_CHA_3 = 3;

    // PWM parameters
    int LEDC_RES_BITS = 12;
    int LEDC_RES = ((1 << LEDC_RES_BITS) - 1);
    int LEDC_FREQ = 500;

    // Status variables
    // These variables can be modified by the website (manual mode) and the behavior tree
    MoveActionMode moveActionMode;
    JawActionMode jawActionMode;
    // When using PID, the controller should read these two variables and calculate desired speed for left and right wheels
    int speed;
    int turnRate;
    // When using PID, the controller should modify these two variables
    int desSpeedL;
    int desSpeedR;
    // For servos, open-loop
    int servoAngleJaw;
    int servoAngleIR;

    // Motor class
    Motor MOTOR_L;// = Motor(MOTOR_L_PWM, MOTOR_L_DIR1, MOTOR_L_DIR2, LEDC_CHA_0, LEDC_RES_BITS, LEDC_FREQ);
    Motor MOTOR_R;// = Motor(MOTOR_R_PWM, MOTOR_R_DIR1, MOTOR_R_DIR2, LEDC_CHA_1, LEDC_RES_BITS, LEDC_FREQ);

    // Servo class
    Servo SERVO_JAW;// = Servo(180, SERVO_JAW_PWM, LEDC_CHA_2, LEDC_RES_BITS, LEDC_FREQ);
    Servo SERVO_IR;// = Servo(180, SERVO_IR_PWM, LEDC_CHA_3, LEDC_RES_BITS, LEDC_FREQ);

    // Constructor with initialization list
    Actions() :
        MOTOR_L(MOTOR_L_PWM, MOTOR_L_DIR1, MOTOR_L_DIR2, LEDC_CHA_0, LEDC_RES_BITS, LEDC_FREQ),
        MOTOR_R(MOTOR_R_PWM, MOTOR_R_DIR1, MOTOR_R_DIR2, LEDC_CHA_1, LEDC_RES_BITS, LEDC_FREQ),
        SERVO_JAW(180, SERVO_JAW_PWM, LEDC_CHA_2, LEDC_RES_BITS, LEDC_FREQ),
        SERVO_IR(180, SERVO_IR_PWM, LEDC_CHA_3, LEDC_RES_BITS, LEDC_FREQ),
        moveActionMode(STOP),
        jawActionMode(JAW_HOLD),
        speed(0),
        turnRate(50),
        desSpeedL(0),
        desSpeedR(0),
        servoAngleJaw(0),
        servoAngleIR(0)
    {
        // ledcSetup(LEDC_CHA_0, LEDC_FREQ, LEDC_RES_BITS);
        // ledcSetup(LEDC_CHA_1, LEDC_FREQ, LEDC_RES_BITS);
        // ledcAttachPin(MOTOR_L_PWM, LEDC_CHA_0);
        // ledcAttachPin(MOTOR_R_PWM, LEDC_CHA_1);
        // ledcSetup(LEDC_CHA_2, LEDC_FREQ, LEDC_RES_BITS);
        // ledcSetup(LEDC_CHA_3, LEDC_FREQ, LEDC_RES_BITS);
        // ledcAttachPin(SERVO_JAW_PWM, LEDC_CHA_2);
        // ledcAttachPin(SERVO_IR_PWM, LEDC_CHA_3);
    }
    // Copy constructor and destructor
    Actions(const Actions& old) = default;
    ~Actions() = default;

    // --- Basic actions ---

    void setMotorSpeed(Motor& MOTOR, int speed)
    {
        // speed is in range [-4095, 4095]
        MOTOR.setSpeed(speed);
    }

    void setServoAngle(Servo& SERVO, int angle)
    {
        // We would like mid angle to be 0 degree, and the servo can rotate ANGLE_RANGE degrees
        // angle is in range [-ANGLE_RANGE/2, ANGLE_RANGE/2]
        SERVO.setAngle(angle);
    }
    
    // --- Upper-layer actions ---
    
    void stop()
    {
        setMotorSpeed(MOTOR_L, 0);
        setMotorSpeed(MOTOR_R, 0);
    }

    void move()  // move according to specified left and right wheel speed
    {
        setMotorSpeed(MOTOR_L, desSpeedL);
        setMotorSpeed(MOTOR_R, desSpeedR);
    }

    void moveForward(int speed)  // with speed parameter
    {
        // speed itself can be positive or negative
        // here we want the car go forward regardless of the +/- sign of speed
        setMotorSpeed(MOTOR_L, abs(speed));
        setMotorSpeed(MOTOR_R, abs(speed));
    }

    void moveForward()  // overload without speed parameter
    {
        setMotorSpeed(MOTOR_L, abs(desSpeedL));
        setMotorSpeed(MOTOR_R, abs(desSpeedR));
    }

    void moveBackward(int speed)
    {
        // speed itself can be positive or negative
        // here we want the car go backward regardless of the +/- sign of speed
        setMotorSpeed(MOTOR_L, -abs(speed));
        setMotorSpeed(MOTOR_R, -abs(speed));
    }

    void moveBackward()  // overload without speed parameter
    {
        setMotorSpeed(MOTOR_L, -abs(desSpeedL));
        setMotorSpeed(MOTOR_R, -abs(desSpeedR));
    }

    void turnLeft(int speed, int turnRate)
    {
        // turnRate is in range [0, 100]
        // Note that when turning left, the right motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)turnRate / 100.0;
        int deltaSpeed = speed * 0.5 * turnRateFloat;
        int speedR = speed + deltaSpeed;
        int speedL = speed - deltaSpeed;
        if (abs(speedR) > LEDC_RES)
        {
            speedR = LEDC_RES * speedR / abs(speedR);  // speedR is LEDC_RES or -LEDC_RES]
            if (speedL != 0)  // If speedL is 0, then we don't need to change it
            {
                speedL = (LEDC_RES - 2 * abs(deltaSpeed)) * speedL / abs(speedL);  // speedL is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        setMotorSpeed(MOTOR_L, speedL);
        setMotorSpeed(MOTOR_R, speedR);
    }

    void turnRight(int speed, int turnRate)
    {
        // turnRate is in range [0, 100]
        // Note that when turning right, the left motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)turnRate / 100.0;
        int deltaSpeed = speed * 0.5 * turnRateFloat;
        int speedL = speed + deltaSpeed;
        int speedR = speed - deltaSpeed;
        if (abs(speedL) > LEDC_RES)
        {
            speedL = LEDC_RES * speedL / abs(speedL);  // speedL is LEDC_RES or -LEDC_RES]
            if (speedR != 0)  // If speedR is 0, then we don't need to change it
            {
                speedR = (LEDC_RES - 2 * abs(deltaSpeed)) * speedR / abs(speedR);  // speedR is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        setMotorSpeed(MOTOR_L, speedL);
        setMotorSpeed(MOTOR_R, speedR);
    }

    void grabTrophy()
    {
        setServoAngle(SERVO_JAW, 90);
    }

    void releaseTrophy()
    {
        setServoAngle(SERVO_JAW, 0);
    }

};

#endif