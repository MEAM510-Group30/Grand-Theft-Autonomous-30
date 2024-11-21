# Grand-Theft-Autonomous-G30
This is the repository for project Grand Theft Autonomous @ MEAM5100 UPenn

## Software Architecture:

  <img width="500" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/cf41ee0e-bd3c-4022-845d-8e9b1f287d1c">
  <br>
  <img width="500" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/91e03efd-31a3-44f9-b2a5-43196c14e472">


## Libraries:

* ESP32Encoder library **v0.8.0 for ESP32-S2**: https://www.arduino.cc/reference/en/libraries/esp32encoder/

* Vive510 library: provided by Prof. Mark Yim

* HTML510 library: provided by Prof. Mark Yim

## Project Report

![示例图片](report/MEAM_510_Final_Project_页面_01.png)
![示例图片](report/MEAM_510_Final_Project_页面_02.png)
![示例图片](report/MEAM_510_Final_Project_页面_03.png)

![示例图片](report/MEAM_510_Final_Project_页面_04.png)
![示例图片](report/MEAM_510_Final_Project_页面_05.png)
![示例图片](report/MEAM_510_Final_Project_页面_06.png)

![示例图片](report/MEAM_510_Final_Project_页面_07.png)
![示例图片](report/MEAM_510_Final_Project_页面_08.png)
![示例图片](report/MEAM_510_Final_Project_页面_09.png)

![示例图片](report/MEAM_510_Final_Project_页面_10.png)
![示例图片](report/MEAM_510_Final_Project_页面_11.png)
![示例图片](report/MEAM_510_Final_Project_页面_12.png)

![示例图片](report/MEAM_510_Final_Project_页面_13.png)
![示例图片](report/MEAM_510_Final_Project_页面_14.png)
![示例图片](report/MEAM_510_Final_Project_页面_15.png)

![示例图片](report/MEAM_510_Final_Project_页面_16.png)
![示例图片](report/MEAM_510_Final_Project_页面_17.png)
![示例图片](report/MEAM_510_Final_Project_页面_18.png)


## Pin Assignment

* 2 motor encoders, 2 each, 4 total
* 2 motor drivers, 1 PWM each, 2 direction pin each, 6 total
* 2 servos, 1 PWM pin each, 2 total
* 2 vive phototransistors, 1 each, 2 total
* 2 ToF sensors, 2 I2C total
* 2 IR photosensors, 1 each, 2 total

* ESP32-S2-DEVKitC-1 and ESP32-C3 Pins

  Link to documentation: https://www.espressif.com/sites/default/files/documentation/esp32-s2-solo-2_esp32-s2-solo-2u_datasheet_en.pdf

  <img width="800" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/70c22510-d1ad-48fb-84ab-99088045204a">


## Motor

* 选取的型号对应12V 130RPM，减速比46.8，电机编码器在电机转一圈输出11个脉冲，因此小车车轮转一圈应输出514.8个脉冲。

  <img width="500" alt="image" src="https://github.com/jbwenjoy/Grand-Theft-Automation-G30/assets/71893666/26b69153-a4a2-46c7-958b-7568aa6c0c8d">

* 但实际测量使用ESP32Encoder v0.8.0 (https://www.arduino.cc/reference/en/libraries/esp32encoder/)的库时，似乎是一圈约987.5个脉冲，因此还是使用987.5

## Bill of Materials:

* Link to Google Docs: https://docs.google.com/spreadsheets/d/1sigmSafWP6qNnaDdqdoLOz4CRqCdhQDDr3Bc3lbWQeE/edit#gid=943090944

