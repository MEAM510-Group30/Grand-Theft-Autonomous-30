# Grand-Theft-Automation-G30
This is the repository for project Grand Theft Automation @ MEAM5100 UPenn

## Pin Assignment

* 2 motor encoders, 2 each, 4 total
* 2 motor drivers, 1 PWM each, 2 direction pin each, 6 total
* 2 servos, 1 PWM pin each, 2 total
* 2 vive phototransistors, 1 each, 2 total
* 2 ToF sensors, 2 I2C total
* 2 IR photosensors, 1 each, 2 total

* ESP32-S2-DEVKitC-1 and ESP32-C3 Pins

  Link to documentation: https://www.espressif.com/sites/default/files/documentation/esp32-s2-solo-2_esp32-s2-solo-2u_datasheet_en.pdf

  <img width="800" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/8c3ced1a-d0aa-4083-8650-6e114b1d28d7">

## Motor

* 选取的型号对应12V 130RPM，减速比46.8，电机编码器在电机转一圈输出11个脉冲，因此小车车轮转一圈应输出514.8个脉冲。

  <img width="500" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/26b69153-a4a2-46c7-958b-7568aa6c0c8d">




## Bill of Materials:

  https://docs.google.com/spreadsheets/d/1sigmSafWP6qNnaDdqdoLOz4CRqCdhQDDr3Bc3lbWQeE/edit#gid=943090944

## Software Architecture:

  <img width="600" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/cf41ee0e-bd3c-4022-845d-8e9b1f287d1c">
