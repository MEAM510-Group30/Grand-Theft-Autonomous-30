/*
    Author:

        @jbwenjoy: Furina de Fontaine

    Description:

        This is the main Arduino file for the ESP32-S2 robot.

*/

#include <stdio.h>
#include <Wire.h>
#include "VL53L0X.h"
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
Actions action = Actions();

// sensors is a global variable containing:
// pin definitions, encoder and vive objects, and sensor readings
Sensors sensors = Sensors();

// behavior is a global variable containing:
// the behavior tree and the action object
Behavior behavior = Behavior();

// communication classes
// web_commun html = web_commun();
// UDP_broadcast udp_police = UDP_broadcast(IPAddress(192, 168, 1, 142));
// UDP_broadcast udp_myPosition = UDP_broadcast(IPAddress(192, 168, 1, 142));
Serial_commun c3(19, 20);
// esp_now esp = esp_now();

#define SENSOR1_SHUTDOWN_PIN 39
#define SENSOR2_SHUTDOWN_PIN 33
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x40

// IR
#define Sensor1 15
#define Sensor2 16

VL53L0X sensor1;
VL53L0X sensor2;

int readTOF(VL53L0X sensor)
{
    int sens;
    sens = sensor.readRangeContinuousMillimeters();
    return sens - 20;
}

int myfreq(int PINUM)
{
    int vol;
    int highvol = 2400;
    int lowvol = 1800;
    unsigned long int curtime;
    unsigned long int t0;
    unsigned long int t1;
    int i = 0;
    while (1)
    {
        vol = analogRead(PINUM);
        curtime = micros();
        if (vol > lowvol)
        {
            break;
        } // break when rising edge
        i++;
        if (i > 500)
        {
            return 999;
        }
    }
    i = 0;
    t0 = curtime;
    while (1)
    {
        vol = analogRead(PINUM);
        if (vol < lowvol)
        {
            break;
        }
        i++;
        if (i > 500)
        {
            return 999;
        }
    }
    i = 0;
    while (1)
    {
        vol = analogRead(PINUM);
        curtime = micros();
        if (vol > lowvol)
        {
            break;
        } // break when rising edge
        i++;
        if (i > 500)
        {
            return 999;
        }
    }
    t1 = curtime;
    unsigned long int period = t1 - t0;
    double frequency = 1000000.0 / period;
    int signalState1;
    if (frequency > 500.0 && frequency < 600.0)
    {
        signalState1 = 1;
    }
    else if (frequency > 20.0 && frequency < 30.0)
    {
        signalState1 = 2;
    }
    else
    {
        signalState1 = 3;
    }
    switch (signalState1)
    {
    case 1:
        // Serial.println("Detected 550Hz signal.");
        return 550;
        break;
    case 2:
        // Serial.println("Detected 23Hz signal.");
        return 23;
        break;
    case 3:
        // Serial.println("\nNo Signal!");
        return 999;
        break;
    }
    Serial.println("Time Out!");
    return 999;
}


void fsm()
{
    if (digitalRead(JAW) == HIGH)
    {
        Serial.println("JAW OPEN");
        behavior.html_state = 'm';
        behavior.action.releaseTrophy();
    }
    else
        Serial.println("JAW CLOSE");
        behavior.html_state = 'm';
        behavior.action.grabTrophy();

    if (digitalRead(action1) == HIGH && digitalRead(action2) == HIGH && digitalRead(action3) == HIGH)
    {
        Serial.println("Forward");
        behavior.html_state = 'm';
        behavior.html_manual_direction = 'f';
    }
    else if (digitalRead(action1) == HIGH && digitalRead(action2) == HIGH && digitalRead(action3) == LOW)
    {
        Serial.println("Backward");
        behavior.html_state = 'm';
        behavior.html_manual_direction = 'b';
    }
    else if (digitalRead(action1) == HIGH && digitalRead(action2) == LOW && digitalRead(action3) == LOW)
    {
        Serial.println("Left");
        behavior.html_state = 'm';
        behavior.html_manual_direction = 'l';
    }
    else if (digitalRead(action1) == HIGH && digitalRead(action2) == LOW && digitalRead(action3) == HIGH)
    {
        Serial.println("Right");
        behavior.html_state = 'm';
        behavior.html_manual_direction = 'r';
    }
    else if (digitalRead(action1) == LOW && digitalRead(action2) == HIGH && digitalRead(action3) == HIGH)
    {
        Serial.println("Stop");
        behavior.html_state = 'm';
        behavior.html_manual_direction = 'o';
    }

    if (digitalRead(mode1) == HIGH && digitalRead(mode2) == HIGH)
    {
        Serial.println("Trophy");
        behavior.html_state = 't';
    }
    else if (digitalRead(mode1) == HIGH && digitalRead(mode2) == LOW)
    {
        Serial.println("Push");
        behavior.html_state = 'p';
    }
    else if (digitalRead(mode1) == LOW && digitalRead(mode2) == HIGH)
    {
        Serial.println("Wall");
        behavior.html_state = 'w';
    }
    else if (digitalRead(mode1) == LOW && digitalRead(mode2) == LOW)
    {
        Serial.println("Manual");
        behavior.html_state = 'm';
    }
}

void setup()
{
    Serial.begin(115200);
    // Setup serial communication
    c3.softSerial.begin(9600);

    // html.initial(IPAddress(192, 168, 1, 142)); // ## To be changed

    
    Wire.begin(19, 20); // SDA on 19, SCL on 20
    pinMode(SENSOR1_SHUTDOWN_PIN, OUTPUT);
    pinMode(SENSOR2_SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SENSOR1_SHUTDOWN_PIN, LOW);
    digitalWrite(SENSOR2_SHUTDOWN_PIN, LOW);


    // Use serial port to communicate with c3 and then with website
    pinMode(JAW, INPUT);     // JAW
    pinMode(action1, INPUT); // action1
    pinMode(action2, INPUT); // action2
    pinMode(action3, INPUT); // action3
    pinMode(mode1, INPUT);   // mode1
    pinMode(mode2, INPUT);   // mode2
    
    // Start tof1
    digitalWrite(SENSOR1_SHUTDOWN_PIN, HIGH);
    sensor1.init(true);

    sensor1.setAddress(SENSOR1_ADDRESS);
    // Start tof2
    digitalWrite(SENSOR2_SHUTDOWN_PIN, HIGH);

    sensor2.init(true);

    sensor2.setAddress(SENSOR2_ADDRESS);
    // Set the sensors to long range mode
    sensor1.setSignalRateLimit(0.1);
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor1.startContinuous();
    sensor2.setSignalRateLimit(0.1);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor2.startContinuous();

    behavior.html_state = 't';
    
    delay(1000);
}

void loop()
{
    fsm();
    
    // put your main code here, to run repeatedly:
    // int tofdis1 = readTOF(sensor1);
    int tofdis2 = readTOF(sensor2);
    Serial.print("\nTOF Front: ");
    Serial.print(tofdis2);
    int irsensor1 = myfreq(Sensor1);
    int irsensor2 = myfreq(Sensor2);
    
    // ### Motor Test Code ###

    // should always be commented when not testing
    // should comment all other code when testing

    // action.moveForward(1000);
    // action.grabTrophy();
    // action.setMotorSpeed(actions.MOTOR_R, 3000);  // right wheel
    // action.setMotorSpeed(actions.MOTOR_L, 3200);  // left wheel
    // behavior.action.turnRightSamePlace(600);

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
    // delay(50);

    // ### Dual Vive Calucation Test Code ###

    // sensors.updateVive();
    // Serial.print('\t');
    // Serial.print(sensors.vive_x_mm);
    // Serial.print('\t');
    // Serial.print(sensors.vive_y_mm);
    // Serial.print('\t');
    // Serial.print(sensors.vive_theta);
    // delay(150);


    // ### PID Same Place Turning Test Code ###

    // sensors.updateEncoder();
    // behavior.action.updateActualSpeed(sensors.speed_L, sensors.speed_R);
    behavior.updateBehaviorClassSensors(tofdis2, 500, sensors.vive_x_mm, sensors.vive_y_mm, sensors.vive_theta, 9999, 9999, irsensor1, irsensor2);


    // // behavior.updateBehaviorClassHTMLVariables(...);
    // // behavior.updateBehaviorClassSensorVariables(...);
    // // behavior.action.updateHTMLData(...);
    // // behavior.action.updateActualSpeed(...);

    // // must be called before taking actions that uses PID speed control
    // action.updateActualSpeed(sensors.speed_L, sensors.speed_R);

    // behavior.action.turnLeftSamePlace();
    // // action.MOTOR_L.setSpeed(2000);
    // // action.MOTOR_R.setSpeed(-2000);

    // Serial.print('\n');
    // Serial.print(sensors.speed_L);
    // Serial.print('\t');
    // Serial.print(sensors.speed_R);
    // Serial.print('\t');
    // Serial.print(action.PIDSpeedL);
    // Serial.print('\t');
    // Serial.print(action.PIDSpeedR);

    // // action.moveBackward(-2000);



    // ### Test Code for moveTo functions ###

    // sensors.updateEncoder();
    // sensors.updateVive();
    // behavior.action.updateActualSpeed(sensors.speed_L, sensors.speed_R);

    // behavior.updateBehaviorClassSensors(500, 500, sensors.vive_x_mm, sensors.vive_y_mm, sensors.vive_theta, 9999, 9999, 9999);

    // behavior.testMoveToFunctionInActionsClass();


    // ### Test code for wall following

    // behavior.html_state = 'w';
    // behavior.tof_front = tofdis2;

    // ### Test code for trophy

    // behavior.html_state = 't';
    behavior.runBehaviorTree();
    
    // // ### Main Code ###
    // // should comment all other test code when using

    // // website cmd checking and global variables updating
    // commun_Mode web_mode = html.mode;
    // commun_Actions web_action = html.action;
    // commun_Jaw web_jaw = html.Jaw;

    // char html_state;
    // char html_manual_direction;
    // bool html_if_jaw_open = true;
    // int html_speed = html.speed;
    // int html_turn_rate = html.turnRate;

    // // map html modes to behavior modes
    // switch (web_mode)
    // {
    // case commun_WALL:
    //     html_state = 'w';
    //     break;
    // case commun_PUSH:
    //     html_state = 'p';
    //     break;
    // case commun_TROPHY:
    //     html_state = 't';
    //     break;
    // case commun_MANUAL:
    //     html_state = 'm';
    //     break;
    // case commun_AUTO:
    //     html_state = 'a';
    //     break;
    // case commun_NOTHING:
    // default:
    //     html_state = 'n';
    //     break;
    // }

    // switch (web_action)
    // {
    // case commun_FORWARD:
    //     html_manual_direction = 'f';
    //     break;
    // case commun_BACKWARD:
    //     html_manual_direction = 'b';
    //     break;
    // case commun_LEFT:
    //     html_manual_direction = 'l';
    //     break;
    // case commun_RIGHT:
    //     html_manual_direction = 'r';
    //     break;
    // case commun_STOP:
    // default:
    //     html_manual_direction = 'o';
    //     break;
    // }

    // switch (web_jaw)
    // {
    // case commun_OPEN:
    //     html_if_jaw_open = true;
    //     break;
    // case commun_CLOSE:
    //     html_if_jaw_open = false;
    //     break;
    // default:
    //     html_if_jaw_open = true;  // default is open
    //     break;
    // }


    // // s2 sensors reading
    // sensors.updateEncoder();
    // sensors.updateVive();

    // // c3 sensors reading through serial communication
    // // c3 data: 2 x ToF data, 2 x IR sensor frequancy, 1 x police car broadcast
    // c3.read();
    // String serial_msg = c3.message;
    // int tof_front = 9999; // default no dectection values
    // int tof_left = 9999;
    // int ir_freq_1 = 999;
    // int ir_freq_2 = 999;
    // int trophy_direction = 999; // should be -180 to 180
    // int police_x_mm = 9999;
    // int police_y_mm = 9999;
    // // TODO: decide how to parse serial_msg
    // // we need to generate 

    // // police position reading through udp broadcast
    // // TODO: decide how to parse udp_msg

    // // behavior variables updating
    // behavior.updateBehaviorClassHTMLVariables(html_state, html_manual_direction, html_if_jaw_open, html_speed, html_turn_rate);
    // behavior.updateBehaviorClassSensors(tof_front, tof_left,
    //                                             sensors.vive_x_mm, sensors.vive_y_mm, sensors.vive_theta,
    //                                             police_x_mm, police_y_mm, trophy_direction);

    // // run behavior tree
    // behavior.runBehaviorTree();
    
    // // delay a little bit
    delay(20);
}
