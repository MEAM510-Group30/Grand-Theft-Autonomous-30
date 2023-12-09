/*
    Authors:

        @jbwenjoy: Furina de Fontaine

    Description:

        This file defines functions for all kinds of actions.

        Three layers of actions should be defined here:
            1. Basic actions, like specifying motor speed, servo angle, etc.
            2. Medium-layer actions, like moving, turning, etc, which are combinations of basic actions.
            2. Upper-layer actions, like wall following, car pushing, etc.

*/

#ifndef ACTIONS_H
#define ACTIONS_H

#include "actions_motor.h"
#include "actions_servo.h"
#include <Arduino.h>
#include "control.h"

enum MoveActionMode
{
    STOP,
    GENERAL_MOVING,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    SAME_PLACE_LEFT,
    SAME_PLACE_RIGHT
};

enum JawActionMode
{
    JAW_HOLD,   // keep the jaw at the current position
    JAW_GRAB,   // grab the trophy
    JAW_RELEASE // release the trophy
};

class Actions // all variables and functions are public
{
public:
    // Motor pins
    int MOTOR_R_PWM = 1;
    int MOTOR_L_PWM = 2;
    int MOTOR_R_DIR2 = 38; // These ports have been fine-tuned for current hardware so that forward is forward, backward is backward.
    int MOTOR_R_DIR1 = 35;
    int MOTOR_L_DIR2 = 36;
    int MOTOR_L_DIR1 = 37;

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
    int IRServoMode; // angle in degrees
    // When using PID, the controller should read these two variables and calculate desired speed for left and right wheels
    int html_speed;
    int html_turnRate;
    // When using PID, the controller should read these two variables, and update PID values
    int desSpeedL;
    int desSpeedR;
    int PIDSpeedL;
    int PIDSpeedR;
    // For servos, open-loop
    int servoAngleJaw;
    int servoAngleIR;

    // Motor class
    Motor MOTOR_L; // = Motor(MOTOR_L_PWM, MOTOR_L_DIR1, MOTOR_L_DIR2, LEDC_CHA_0, LEDC_RES_BITS, LEDC_FREQ);
    Motor MOTOR_R; // = Motor(MOTOR_R_PWM, MOTOR_R_DIR1, MOTOR_R_DIR2, LEDC_CHA_1, LEDC_RES_BITS, LEDC_FREQ);

    // Servo class
    Servo SERVO_JAW; // = Servo(180, SERVO_JAW_PWM, LEDC_CHA_2, LEDC_RES_BITS, LEDC_FREQ);
    Servo SERVO_IR;  // = Servo(180, SERVO_IR_PWM, LEDC_CHA_3, LEDC_RES_BITS, LEDC_FREQ);

    // PID class
    PIDController PID_L; // = PIDController(kp=15.0, ki=0.05, kd=0.05);
    PIDController PID_R; // = PIDController(kp=15.0, ki=0.05, kd=0.05);
    PIDController PID_wall;
    // PIDController PID_pos_x;
    // PIDController PID_pos_y;

    // Actual speedL/R in mm/s, this should be updated by main in every loop
    float ACTUAL_SPEED_L;
    float ACTUAL_SPEED_R;

    // ToF values, in mm
    int tof_side;
    int tof_front;

    // Position values, in mm
    float current_x;
    float current_y;
    float current_theta;
    float target_x;
    float target_y;
    float target_theta;

    // For moveToPosition function
    bool turnToHeadingFinished;

    // Constructor with initialization list
    Actions() : MOTOR_L(MOTOR_L_PWM, MOTOR_L_DIR1, MOTOR_L_DIR2, LEDC_CHA_0, LEDC_RES_BITS, LEDC_FREQ),
                MOTOR_R(MOTOR_R_PWM, MOTOR_R_DIR1, MOTOR_R_DIR2, LEDC_CHA_1, LEDC_RES_BITS, LEDC_FREQ),
                SERVO_JAW(180, SERVO_JAW_PWM, LEDC_CHA_2, LEDC_RES_BITS, LEDC_FREQ),
                SERVO_IR(180, SERVO_IR_PWM, LEDC_CHA_3, LEDC_RES_BITS, LEDC_FREQ),
                PID_L(), PID_R(),                       // use the default PID parameters in control.h for speed control
                PID_wall(0.1, 0.0, 0.0), // wall following PID parameters
                // PID_pos_x(kp = 0.1, ki = 0.0, kd = 0.0), PID_pos_y(kp = 0.1, ki = 0.0, kd = 0.0),
                ACTUAL_SPEED_L(0), ACTUAL_SPEED_R(0),
                moveActionMode(STOP), jawActionMode(JAW_HOLD),
                html_speed(0), html_turnRate(50),
                desSpeedL(0), desSpeedR(0), PIDSpeedL(0), PIDSpeedR(0),
                servoAngleJaw(0), servoAngleIR(0),
                IRServoMode(0),
                tof_side(0), tof_front(0),
                current_x(0), current_y(0), current_theta(0),
                target_x(0), target_y(0), target_theta(0),
                turnToHeadingFinished(false)
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
    Actions(const Actions &old) = default;
    ~Actions() = default;

    // --- Basic actions ---

    void setMotorSpeed(Motor &MOTOR, int speed)
    {
        // speed is in range [-4095, 4095]
        MOTOR.setSpeed(speed);
    }

    void setServoAngle(Servo &SERVO, int angle)
    {
        // For the IR sensor servo, we would like mid angle to be 0 degree, and the servo can rotate ANGLE_RANGE degrees
        // so angle is [-ANGLE_RANGE/2, ANGLE_RANGE/2]
        // For the jaw servo, when jaw is open the angle is...
        // when jaw is closed the angle is...
        SERVO.setAngle(angle);
    }

    void updateHTMLData(int speed, int turnRate)
    {
        // Should be called by mainloop/behavior.h every time before using html_speed and html_turnRate
        html_speed = speed;
        html_turnRate = turnRate;
    }

    void updateActualSpeed(float speed_L, float speed_R)
    {
        // Should be called in main loop before using actual speed (eg. in PID) every time
        ACTUAL_SPEED_L = speed_L;
        ACTUAL_SPEED_R = speed_R;
    }

    void updateToFValues(int tof_side, int tof_front)
    {
        // Should be called in main loop before using tof values every time
    }

    // --- Medium-layer motor actions ---

    void stop()
    {
        setMotorSpeed(MOTOR_L, 0);
        setMotorSpeed(MOTOR_R, 0);
        moveActionMode = STOP;
    }

    void move() // move according to specified left and right wheel speed
    {
        desSpeedL = html_speed;
        desSpeedR = html_speed;
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
        moveActionMode = GENERAL_MOVING;
    }

    void moveForward(int speed) // not using html_speed
    {
        // with speed parameter, however, this function does not have feedback control,
        // therefore shouldn't be used directly in checkoff
        // speed itself can be positive or negative
        // here we want the car go forward regardless of the +/- sign of speed
        setMotorSpeed(MOTOR_L, abs(speed));
        setMotorSpeed(MOTOR_R, abs(speed));
        moveActionMode = MOVE_FORWARD;
    }

    void moveForward() // using html_speed
    {
        desSpeedL = abs(html_speed);
        desSpeedR = abs(html_speed);
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, abs(PIDSpeedL));
        setMotorSpeed(MOTOR_R, abs(PIDSpeedR));
        moveActionMode = MOVE_FORWARD;
    }

    void moveBackward(int speed) // not using html_speed
    {
        // with speed parameter, however, this function does not have feedback control,
        // therefore shouldn't be used directly in checkoff
        // speed itself can be positive or negative
        // here we want the car go backward regardless of the +/- sign of speed
        setMotorSpeed(MOTOR_L, -abs(speed));
        setMotorSpeed(MOTOR_R, -abs(speed));
        moveActionMode = MOVE_BACKWARD;
    }

    void moveBackward() // using html_speed
    {
        desSpeedL = -abs(html_speed);
        desSpeedR = -abs(html_speed);
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, -abs(desSpeedL));
        setMotorSpeed(MOTOR_R, -abs(desSpeedR));
        moveActionMode = MOVE_BACKWARD;
    }

    void turnLeft(int speed, int turnRate) // not using html_speed and html_turnRate
    {
        // turnRate is in range [0, 100]
        // Note that when turning left, the right motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)turnRate / 100.0;
        int deltaSpeed = speed * 0.5 * turnRateFloat;
        int speedR = speed + deltaSpeed;
        int speedL = speed - deltaSpeed;
        if (abs(speedR) > LEDC_RES)
        {
            speedR = LEDC_RES * speedR / abs(speedR); // speedR is LEDC_RES or -LEDC_RES]
            if (speedL != 0)                          // If speedL is 0, then we don't need to change it
            {
                speedL = (LEDC_RES - 2 * abs(deltaSpeed)) * speedL / abs(speedL); // speedL is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        desSpeedL = speedL;
        desSpeedR = speedR;
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
        moveActionMode = TURN_LEFT;
    }

    void turnLeft() // using html_speed and html_turnRate
    {
        // turnRate is in range [0, 100]
        // Note that when turning left, the right motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)html_turnRate / 100.0;
        int deltaSpeed = html_speed * 0.5 * turnRateFloat;
        int speedR = html_speed + deltaSpeed;
        int speedL = html_speed - deltaSpeed;
        if (abs(speedR) > LEDC_RES)
        {
            speedR = LEDC_RES * speedR / abs(speedR); // speedR is LEDC_RES or -LEDC_RES]
            if (speedL != 0)                          // If speedL is 0, then we don't need to change it
            {
                speedL = (LEDC_RES - 2 * abs(deltaSpeed)) * speedL / abs(speedL); // speedL is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        desSpeedL = speedL;
        desSpeedR = speedR;
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
        moveActionMode = TURN_LEFT;
    }

    void turnRight(int speed, int turnRate) // not using html_speed and html_turnRate
    {
        // turnRate is in range [0, 100]
        // Note that when turning right, the left motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)turnRate / 100.0;
        int deltaSpeed = speed * 0.5 * turnRateFloat;
        int speedL = speed + deltaSpeed;
        int speedR = speed - deltaSpeed;
        if (abs(speedL) > LEDC_RES)
        {
            speedL = LEDC_RES * speedL / abs(speedL); // speedL is LEDC_RES or -LEDC_RES]
            if (speedR != 0)                          // If speedR is 0, then we don't need to change it
            {
                speedR = (LEDC_RES - 2 * abs(deltaSpeed)) * speedR / abs(speedR); // speedR is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        desSpeedL = speedL;
        desSpeedR = speedR;
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
        moveActionMode = TURN_RIGHT;
    }

    void turnRight() // using html_speed and html_turnRate
    {
        // turnRate is in range [0, 100]
        // Note that when turning right, the left motor should always rotate faster, regardless of the +/- sign of speed
        float turnRateFloat = (float)html_turnRate / 100.0;
        int deltaSpeed = html_speed * 0.5 * turnRateFloat;
        int speedL = html_speed + deltaSpeed;
        int speedR = html_speed - deltaSpeed;
        if (abs(speedL) > LEDC_RES)
        {
            speedL = LEDC_RES * speedL / abs(speedL); // speedL is LEDC_RES or -LEDC_RES]
            if (speedR != 0)                          // If speedR is 0, then we don't need to change it
            {
                speedR = (LEDC_RES - 2 * abs(deltaSpeed)) * speedR / abs(speedR); // speedR is LEDC_RES - 2 * abs(deltaSpeed) or -LEDC_RES + 2 * abs(deltaSpeed)
            }
        }
        desSpeedL = speedL;
        desSpeedR = speedR;
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
        moveActionMode = TURN_RIGHT;
    }

    void turnLeftSamePlace(int speed) // turn left without moving forward or backward
    {
        // speed (duty cycle) itself can be positive or negative
        // here we want the car turn left regardless of the +/- sign of speed
        desSpeedL = -abs(speed);
        desSpeedR = abs(speed);

        // Serial.print('\n');
        // Serial.print(desSpeedL);
        // Serial.print('\t');
        // Serial.print(desSpeedR);

        PIDSpeedCalibration();

        // Serial.print('\n');
        // Serial.print(PIDSpeedL);
        // Serial.print('\t');
        // Serial.print(PIDSpeedR);

        setMotorSpeed(MOTOR_L, -abs(PIDSpeedL));
        setMotorSpeed(MOTOR_R, abs(PIDSpeedR));
        moveActionMode = SAME_PLACE_LEFT;
    }

    void turnLeftSamePlace() // overload without speed parameter, with PID
    {
        desSpeedL = -abs(html_speed);
        desSpeedR = abs(html_speed);
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, -abs(PIDSpeedL));
        setMotorSpeed(MOTOR_R, abs(PIDSpeedR));
        moveActionMode = SAME_PLACE_LEFT;
    }

    void turnRightSamePlace(int speed) // turn right without moving forward or backward
    {
        // speed itself can be positive or negative
        // here we want the car turn right regardless of the +/- sign of speed
        desSpeedL = abs(speed);
        desSpeedR = -abs(speed);

        // Serial.print('\n');
        // Serial.print(desSpeedL);
        // Serial.print('\t');
        // Serial.print(desSpeedR);

        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, abs(PIDSpeedL));
        setMotorSpeed(MOTOR_R, -abs(PIDSpeedR));
        moveActionMode = SAME_PLACE_RIGHT;
    }

    void turnRightSamePlace() // overload without speed parameter
    {
        desSpeedL = abs(html_speed);
        desSpeedR = -abs(html_speed);
        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, abs(PIDSpeedL));
        setMotorSpeed(MOTOR_R, -abs(PIDSpeedR));
        moveActionMode = SAME_PLACE_RIGHT;
    }

    void PIDSpeedCalibration() // This function generates the des speed for left and right wheels in duty cycles
    {
        // This function reads the movement status, the speed, and the turn rate, and calculates the desired speed for left and right wheels
        // The desired speed is stored in desSpeedL and desSpeedR
        // This function should be called after calculating desSpeedL/R and before setMotorSpeed in all moving functions

        // First we need to know the map between the actual speed in mm/s and the duty cycle
        // During no-load test, when duty cycle is set to 3000/4095, the actual speed for left wheel is 325mm/s and right wheel 314mm/s
        // Considering the friction, we can assume that the actual speed in mm/s is 0.08 times the duty cycle
        float desSpeedL_mm_s = (float)desSpeedL * 0.08;
        float desSpeedR_mm_s = (float)desSpeedR * 0.08;

        // Serial.print('\n');
        // Serial.print(desSpeedL_mm_s);
        // Serial.print('\t');
        // Serial.print(desSpeedR_mm_s);

        PIDSpeedL = PID_L.PID(desSpeedL_mm_s, ACTUAL_SPEED_L);
        PIDSpeedR = PID_R.PID(desSpeedR_mm_s, ACTUAL_SPEED_R);
    }

    // --- Jaw actions ---

    void grabTrophy()
    {
        setServoAngle(SERVO_JAW, 80);
        jawActionMode = JAW_GRAB;
    }

    void releaseTrophy()
    {
        setServoAngle(SERVO_JAW, 30);
        jawActionMode = JAW_RELEASE;
    }

    // --- IR servo actions ---

    void setIRServoAngle(int angle)
    {
        setServoAngle(SERVO_IR, angle);
        IRServoMode = angle;
    }

    // --- Upper-layer actions ---

    void followWallForward()
    {
        // Wall following PID uses the distance to the wall as input,
        // a constant distance (200mm) as the reference,
        // and the output is the error in desSpeed for left and right wheels

        // As we want the side ToF sensor to be at the left side of the robot,
        // if the robot is too close to the wall, the left wheel should rotate faster,
        // if the robot is too far from the wall, the right wheel should rotate faster.

        float refernce_distance = 150; // mm

        float LminusR = PID_wall.PID(refernce_distance, tof_side); // duty cycle
        float LplusR = 2 * html_speed;                             // duty cycle
        desSpeedL = (LplusR + LminusR) / 2;
        desSpeedR = (LplusR - LminusR) / 2;

        PIDSpeedCalibration();
        setMotorSpeed(MOTOR_L, PIDSpeedL);
        setMotorSpeed(MOTOR_R, PIDSpeedR);
    }

    void turnToHeading(float heading, float threshold=5.0)
    {
        // Turn to the specified heading
        // The heading is in degrees, and 0 degree is the x+ axis

        float delta_theta = heading - current_theta;
        if (delta_theta > 180)
        {
            delta_theta -= 360;
        }
        else if (delta_theta < -180)
        {
            delta_theta += 360;
        }

        if (delta_theta > threshold)
        {
            turnLeftSamePlace();
        }
        else if (delta_theta < -threshold)
        {
            turnRightSamePlace();
        }
        else
        {
            stop();
        }

    }

    void moveToPosition(float x, float y, float threshold=10.0, float angle_threshold=5.0)
    {
        // Write the target position to target_x and target_y
        target_x = x;
        target_y = y;        
        target_theta = atan2(target_y - current_y, target_x - current_x) * 180 / PI;

        if (abs(current_theta - target_theta) > angle_threshold) // Head towards the target position
        {
            turnToHeading(target_theta, angle_threshold);
        }
        else // Move forward
        {
            if (((current_x - target_x) * (current_x - target_x) + (current_y - target_y) * (current_y - target_y) > threshold * threshold))
            {
                moveForward();
            }
            else
            {
                stop();
            }
        }
    }

};

#endif