/*
    Discription:

        In this file, we define the functions that are used to read the sensor data from the ESP32-C3.

        Sensors include two ToF sensors and the two IR photosensors installed on the servo.

        The ToF sensors are used to detect the distance between the robot and the wall, return type should be float in millimeters.

        The IR photosensors are used to detect the direction when the trophy is roughly in front of the robot,
        return type should be two integers, one for each sensor, telling whether the sensor detects a signal, and which frequency the signal is.

*/

#ifndef C3_SENSORS_H
#define C3_SENSORS_H

#include <Wire.h>
#include <VL53L0X.h>

class senors
{
    VL53L0X sensor;

    sensors()
    {
        Serial.begin(9600);
        Wire.begin();

        sensor.setTimeout(500);
        if (!sensor.init())
        {
            Serial.println("Failed to detect and initialize sensor!");
            while (1)
            {
            }
        }
        sensor.startContinuous();
    }

    ~sensors() {}

    uint16_t read()
    {
        uint16_t readValue = sensor.readRangeContinuousMillimeters();
        Serial.print(readValue);
        if (sensor.timeoutOccurred())
        {
            Serial.print(" TIMEOUT");
        }
    }
    return readValue;
}

#endif
