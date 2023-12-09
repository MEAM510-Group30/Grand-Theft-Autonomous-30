#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>

// Pins for the shutdown pins on the sensors
#define SENSOR1_SHUTDOWN_PIN 18
#define SENSOR2_SHUTDOWN_PIN 19

// Addresses for the sensors
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

#define Sensor1 1
#define Sensor2 0
float frequency1,frequency2;
VL53L0X sensor1;
VL53L0X sensor2;
//IR
const int thresholdHigh = 3550; // 高电平阈值
const int thresholdLow = 1300; // 低电平阈值
const unsigned long minPulseWidth = 100; // 最短脉冲宽度，以微秒为单位，以过滤噪声
const int confirmCount = 3; // 需要连续测量到相同信号的次数
int signalState1 = 0; // 0: 未检测, 1: 550Hz, 2: 25Hz, 3: 范围外
int lastSignalState1 = 0;
int confirmSignalCount1 = 0;
int signalState2 = 0; // 0: 未检测, 1: 550Hz, 2: 25Hz, 3: 范围外
int lastSignalState2 = 0;
int confirmSignalCount2 = 0;

int readTOF(VL53L0X sensor);
int getfreq1();
int getfreq2();

void setup() {
  // put your setup code here, to run once:
  //IR
  Serial.begin(115200); //
  pinMode(Sensor1,INPUT); //
  pinMode(Sensor2,INPUT); //

  //TOF
  Wire.begin(6, 7); // SDA on 6, SCL on 7
  pinMode(SENSOR1_SHUTDOWN_PIN, OUTPUT);
  pinMode(SENSOR2_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SENSOR1_SHUTDOWN_PIN, LOW);
  digitalWrite(SENSOR2_SHUTDOWN_PIN, LOW);
  delay(10);
  // Start tof1
  digitalWrite(SENSOR1_SHUTDOWN_PIN, HIGH);
  delay(10);
  sensor1.init(true);
  delay(10);
  sensor1.setAddress(SENSOR1_ADDRESS);
  // Start tof2
  digitalWrite(SENSOR2_SHUTDOWN_PIN, HIGH);
  delay(10);
  sensor2.init(true);
  delay(10);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  //getnumber
  int tofdis1 = readTOF(sensor1);
  int tofdis2 = readTOF(sensor1);
  int irsensor1 = getfreq1();
  int irsensor2 = getfreq2();
  //print
  /*
  Serial.println("\nTOF Front: ");
  Serial.print(readTOF(sensor1));
  Serial.println("\nTOF Side: ");
  Serial.print(readTOF(sensor2));
  Serial.println("\nFrequency of IRSensor Right: ");
  Serial.print(getfreq1());
  Serial.println("\nFrequency of IRSensor Left: ");
  Serial.print(getfreq2());
  Serial.println('\n');
  delay(250);
  */

}

int readTOF(VL53L0X sensor){
  int sens;
  sens=sensor.readRangeContinuousMillimeters();
  return sens-20;
}

int getfreq1(){
  int sensorPin = Sensor1;
  int coun = 0;
  while(1){
  coun++;
  if(coun >= 10000){Serial.println("\nTime out!");return 999;}
  static unsigned long lastRise = 0; // 上升沿时间
  static unsigned long highWidth = 0; // 高电平宽度
  static unsigned long lowWidth = 0; // 低电平宽度
  static bool isHigh = false; // 当前是否是高电平

  int value = analogRead(sensorPin); // 读取传感器值

  // 检测高电平
  if (value > thresholdHigh && !isHigh) {
    unsigned long now = micros();
    lowWidth = now - lastRise; // 计算低电平宽度
    lastRise = now;
    isHigh = true;
  }

  // 检测低电平
  if (value < thresholdLow && isHigh) {
    unsigned long now = micros();
    highWidth = now - lastRise; // 计算高电平宽度

    // 检查脉冲宽度是否有效，以避免噪声干扰
    if (highWidth > minPulseWidth && lowWidth > minPulseWidth) {
      // 计算频率
      unsigned long period = highWidth + lowWidth;
      float frequency = 1000000.0 / period;
      
      // 根据频率区分信号
      if (frequency > 500.0 && frequency < 600.0) {
        signalState1 = 1;
      } else if (frequency > 20.0 && frequency < 30.0) {
        signalState1 = 2;
      } else {
        signalState1 = 3;
      }

      // 检查是否连续检测到相同的信号
      if (signalState1 == lastSignalState1) {
        confirmSignalCount1++;
        if (confirmSignalCount1 >= confirmCount) {
          switch (signalState1) {
            case 1:
              Serial.println("Detected 550Hz signal.");
              lastSignalState1 = signalState1;
              lastRise = now;
              isHigh = false;
              return 550;
              break;
            case 2:
              Serial.println("Detected 23Hz signal.");
              lastSignalState1 = signalState1;
              lastRise = now;
              isHigh = false;
              return 25;
              break;
            case 3:
              Serial.println("\nNo Signal!");
              lastSignalState1 = signalState1;
              lastRise = now;
              isHigh = false;
              return 999;
              break;
          }
        }
      } else {
        confirmSignalCount1 = 0;
      }
      lastSignalState1 = signalState1;
    }
    lastRise = now;
    isHigh = false;
  }
}
}

int getfreq2(){
  int sensorPin = Sensor2;
  int coun = 0;
  while(1){
  coun++;
  if(coun >= 10000){Serial.println("\nTime out!");return 999;}
  static unsigned long lastRise = 0; // 上升沿时间
  static unsigned long highWidth = 0; // 高电平宽度
  static unsigned long lowWidth = 0; // 低电平宽度
  static bool isHigh = false; // 当前是否是高电平

  int value = analogRead(sensorPin); // 读取传感器值

  // 检测高电平
  if (value > thresholdHigh && !isHigh) {
    unsigned long now = micros();
    lowWidth = now - lastRise; // 计算低电平宽度
    lastRise = now;
    isHigh = true;
  }

  // 检测低电平
  if (value < thresholdLow && isHigh) {
    unsigned long now = micros();
    highWidth = now - lastRise; // 计算高电平宽度

    // 检查脉冲宽度是否有效，以避免噪声干扰
    if (highWidth > minPulseWidth && lowWidth > minPulseWidth) {
      // 计算频率
      unsigned long period = highWidth + lowWidth;
      float frequency = 1000000.0 / period;
      
      // 根据频率区分信号
      if (frequency > 500.0 && frequency < 600.0) {
        signalState2 = 1;
      } else if (frequency > 20.0 && frequency < 30.0) {
        signalState2 = 2;
      } else {
        signalState2 = 3;
      }

      // 检查是否连续检测到相同的信号
      if (signalState2 == lastSignalState2) {
        confirmSignalCount2++;
        if (confirmSignalCount2 >= confirmCount) {
          switch (signalState2) {
            case 1:
              Serial.println("Detected 550Hz signal.");
              lastSignalState2 = signalState2;
              lastRise = now;
              isHigh = false;
              return 550;
              break;
            case 2:
              Serial.println("Detected 23Hz signal.");
              lastSignalState2 = signalState2;
              lastRise = now;
              isHigh = false;
              return 25;
              break;
            case 3:
              Serial.println("\nNo Signal!");
              lastSignalState2 = signalState2;
              lastRise = now;
              isHigh = false;
              return 999;
              break;
          }
        }
      } else {
        confirmSignalCount2 = 0;
      }
      lastSignalState2 = signalState2;
    }
    lastRise = now;
    isHigh = false;
  }
}
}