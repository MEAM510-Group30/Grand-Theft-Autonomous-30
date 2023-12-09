/*
    Author:

        @jbwenjoy: Furina de Fontaine

    Description:

        This is the main Arduino file for the ESP32-S2 robot.

*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "html510.h"

#include "body.h"

#include "behavior.h"

#include "control.h"

#include "communication.h"

#include "s2_sensors.h"

#include "actions.h"
#include "actions_motor.h"
#include "actions_servo.h"

// actions is a global variable containing:
// pin definitions, ledc pwm parameters, two motor and two servo objects,
// and basic actions of specifying motor speeds and servo angles
// This gloabl variable is only for debugging purposes
// In the final version, the actions object should be created in the behavior class
// Actions actions = Actions();

// sensors is a global variable containing:
// pin definitions, encoder and vive objects, and sensor readings
Sensors sensors = Sensors();

// behavior is a global variable containing:
// the behavior tree and the action object
Behavior behavior = Behavior();

// communication classes
web_commun html = web_commun();
UDP_broadcast udp_police = UDP_broadcast(IPAddress(192, 168, 1, 142));
UDP_broadcast udp_myPosition = UDP_broadcast(IPAddress(192, 168, 1, 142));
Serial_commun c3(19, 20);
esp_now esp = esp_now();

void setup()
{
    Serial.begin(115200);
    // Setup serial communication
    c3.softSerial.begin(9600);

    html.initial(IPAddress(192, 168, 1, 142)); // ## To be changed
}

void loop()
{
    // ### Motor Test Code ###

    // should always be commented when not testing
    // should comment all other code when testing

    // actions.moveForward(1000);
    // actions.setMotorSpeed(actions.MOTOR_R, 3000);  // right wheel
    // actions.setMotorSpeed(actions.MOTOR_L, 3000);  // left wheel

    // ### Single Vive Test Code ###

    // sensors.updateVive();
    // Serial.print('\n');
    // Serial.print(sensors.vive1_x);
    // Serial.print('\t');
    // Serial.print(sensors.vive1_y);
    // delay(200);

    // ### Dual Vive Test Code ###

    // sensors.updateVive();
    // Serial.print('\n');
    // Serial.print(sensors.vive1_x);
    // Serial.print('\t');
    // Serial.print(sensors.vive1_y);
    // Serial.print('\t');
    // Serial.print(sensors.vive2_x);
    // Serial.print('\t');
    // Serial.print(sensors.vive2_y);
    // delay(200);

    // ### PID Same Place Turning Test Code ###

    // sensors.updateEncoder();
    // actions.updateActualSpeed(sensors.speed_L, sensors.speed_R);

    // // behavior.updateBehaviorClassHTMLVariables(...);
    // // behavior.updateBehaviorClassSensorVariables(...);
    // // behavior.action.updateHTMLData(...);
    // // behavior.action.updateActualSpeed(...);

    // // must be called before taking actions that uses PID speed control
    // actions.updateActualSpeed(sensors.speed_L, sensors.speed_R);

    // actions.turnLeftSamePlace(4000);
    // // actions.MOTOR_L.setSpeed(2000);
    // // actions.MOTOR_R.setSpeed(-2000);

    // Serial.print('\n');
    // Serial.print(sensors.speed_L);
    // Serial.print('\t');
    // Serial.print(sensors.speed_R);
    // Serial.print('\t');
    // Serial.print(actions.PIDSpeedL);
    // Serial.print('\t');
    // Serial.print(actions.PIDSpeedR);

    // // actions.moveBackward(-2000);

    // delay(50);

    // ### Main Code ###
    // should comment all other test code when using

    // website cmd checking and global variables updating
    commun_Mode web_mode = html.mode;
    commun_Actions web_action = html.action;
    commun_Jaw web_jaw = html.Jaw;

    char html_state;
    char html_manual_direction;
    bool html_if_jaw_open = true;
    int html_speed = html.speed;
    int html_turn_rate = html.turnRate;

    // map html modes to behavior modes
    switch (web_mode)
    {
    case commun_WALL:
        html_state = 'w';
        break;
    case commun_PUSH:
        html_state = 'p';
        break;
    case commun_TROPHY:
        html_state = 't';
        break;
    case commun_MANUAL:
        html_state = 'm';
        break;
    case commun_AUTO:
        html_state = 'a';
        break;
    case commun_NOTHING:
    default:
        html_state = 'n';
        break;
    }

    switch (web_action)
    {
    case commun_FORWARD:
        html_manual_direction = 'f';
        break;
    case commun_BACKWARD:
        html_manual_direction = 'b';
        break;
    case commun_LEFT:
        html_manual_direction = 'l';
        break;
    case commun_RIGHT:
        html_manual_direction = 'r';
        break;
    case commun_STOP:
    default:
        html_manual_direction = 'o';
        break;
    }

    switch (web_jaw)
    {
    case commun_OPEN:
        html_if_jaw_open = true;
        break;
    case commun_CLOSE:
        html_if_jaw_open = false;
        break;
    default:
        html_if_jaw_open = true;  // default is open
        break;
    }


    // s2 sensors reading
    sensors.updateEncoder();
    sensors.updateVive();

    // c3 sensors reading through serial communication
    // c3 data: 2 x ToF data, 2 x IR sensor frequancy, 1 x police car broadcast
    c3.read();
    String serial_msg = c3.msg;
    int tof_front = 9999; // default no dectection values
    int tof_left = 9999;
    int ir_freq_1 = 999;
    int ir_freq_2 = 999;
    int trophy_direction = 999; // should be -180 to 180
    int police_x_mm = 9999;
    int police_y_mm = 9999;
    // TODO: decide how to parse serial_msg
    // we need to generate 

    // police position reading through udp broadcast
    // TODO: decide how to parse udp_msg

    // behavior variables updating
    behavior.updateBehaviorClassHTMLVariables(html_state, html_manual_direction, html_if_jaw_open, html_speed, html_turn_rate);
    behavior.updateBehaviorClassSensors(tof_front, tof_left,
                                                sensors.vive_x_mm, sensors.vive_y_mm, sensors.vive_theta,
                                                police_x_mm, police_y_mm, trophy_direction);

    // run behavior tree
    behavior.runBehaviorTree();
    //

    delay(50);
}
