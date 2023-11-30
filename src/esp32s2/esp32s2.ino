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

#include "website.h"

#include "behavior.h"

#include "control.h"

#include "communication.h"

#include "s2_sensors.h"
#include "s2_sensors_encoder.h"
#include "s2_sensors_vive.h"

#include "actions.h"
#include "actions_motor.h"
#include "actions_servo.h"


// actions is a global variable containing: 
// pin definitions, ledc pwm parameters, two motor and two servo objects, 
// and basic actions of specifying motor speeds and servo angles
Actions actions = Actions();

void setup() {
    Serial.begin(115200);

}

void loop() {
    // // ### Motor Test Code ###
    // // should always be commented when not testing
    // // should comment all other code when testing
    // actions.MOTOR_L.setSpeed(2000);
    // delay(200);

    // ### Main Code ###
    // should comment all other test code when using

    // website cmd checking and global variables updating


    // sensor reading and global variables updating


    // run behavior tree, generate action list

    // 

}
