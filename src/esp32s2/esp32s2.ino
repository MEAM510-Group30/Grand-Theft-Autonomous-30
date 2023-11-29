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



Actions actions = Actions();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

}

void loop() {
    // put your main code here, to run repeatedly:
    actions.MOTOR_L.setSpeed(2000);
}
