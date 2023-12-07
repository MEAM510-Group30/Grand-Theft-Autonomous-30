// I'm still thinking about whether this function should be made a seperete header file or just within the ino file.

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <iostream>
#include <cstdint>
#include <esp_now.h>
#include <SoftwareSerial.h>
#include "body.h"
#include "html510.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

enum commun_Mode
{
    commun_WALL,
    commun_PUSH,
    commun_TROPHY,
    commun_MANUAL,
    commun_NOTHING
};

enum commun_Actions
{
    commun_FORWARD,
    commun_BACKWARD,
    commun_LEFT,
    commun_RIGHT,
    commun_STOP
};

class esp_now
{                 // use member function sendMessage() to send group number
#define CHANNEL 1 // channel can be 1 to 14, channel 0 means current channel.
#define MAC_RECV                           \
    {                                      \
        0x84, 0xF7, 0x03, 0xA8, 0xBE, 0x30 \
    } // receiver MAC address

public:
    int groupNumber = 30;
    uint8_t message[5];
    esp_now_peer_info_t peer1 = {
        .peer_addr = MAC_RECV,
        .channel = CHANNEL,
        .encrypt = false,
    };

    esp_now()
    {
        Serial.begin(115200);
        WiFi.mode(WIFI_STA);
        Serial.print("Sending MAC: ");
        Serial.println(WiFi.macAddress());
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("init failed");
            ESP.restart();
        }

        // esp_now_register_send_cb(OnDataSent); // optional if you want ack interrupt

        if (esp_now_add_peer(&peer1) != ESP_OK)
        {
            Serial.println("Pair failed"); // ERROR  should not happen
        }
        sprintf((char *)message, "%d ", groupNumber);
    }

    ~esp_now() {}

    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if (status == ESP_NOW_SEND_SUCCESS)
            Serial.println("Success ");
        else
            Serial.println("Fail ");
    }

    void sendMessage()
    { // use in main function to send group number every time communicate with web       // message is the group number
        if (esp_now_send(peer1.peer_addr, message, sizeof(message)) == ESP_OK)
            Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n", message, peer1.peer_addr[0], peer1.peer_addr[1], peer1.peer_addr[2], peer1.peer_addr[3], peer1.peer_addr[4], peer1.peer_addr[5]);
        else
            Serial.println("Send failed");
    }
};

class web_commun
{ // in main function, initialize a web_commun variable to attach handlers
  // visit commun_Mode, Action, speed, turnRate in main function
public:
    static esp_now esp_now_message;
    static commun_Mode mode;
    static commun_Actions action;
    static int speed;
    static int turnRate;
    static WiFiServer wifi_server;
    static HTML510Server html_server;

    web_commun(IPAddress local_IP, const char *ssid = "Furina", const char *pwd = "Furinaaa")
    {
        // our own IP address is local_IP above
        IPAddress gateway_IP(192, 168, 1, 1);
        IPAddress subnet_IP(255, 255, 255, 0);

        WiFi.mode(WIFI_MODE_AP); // wifi in ap mode, no router
        WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP);
        WiFi.softAP(ssid, pwd);
        wifi_server.begin();
        IPAddress softAP_IP = WiFi.softAPIP();

        html_server.begin();
        html_server.attachHandler("/", handleRoot);
        html_server.attachHandler("/tro", handleTrophy);
        html_server.attachHandler("/man", handleManual);
        html_server.attachHandler("/wall", handleWall);
        html_server.attachHandler("/car", handleCar);
        html_server.attachHandler("/F", handleForward);
        html_server.attachHandler("/B", handleBackward);
        html_server.attachHandler("/L", handleLeft);
        html_server.attachHandler("/R", handleRight);
        html_server.attachHandler("/O", handleStop);
        html_server.attachHandler("/speed_slider=", handleSpeed);
        html_server.attachHandler("/turn_rate_slider=", handleTurnRate);
    }

    ~web_commun() {}

    static void handleRoot()
    {
        html_server.sendhtml(body);
    }

    static void handleTrophy()
    {
        esp_now_message.sendMessage();
        mode = commun_TROPHY;
    }

    static void handleManual()
    {
        esp_now_message.sendMessage();
        mode = commun_MANUAL;
    }

    static void handleWall()
    {
        esp_now_message.sendMessage();
        mode = commun_WALL;
    }

    static void handleCar()
    {
        esp_now_message.sendMessage();
        mode = commun_PUSH;
    }

    static void handleForward()
    {
        esp_now_message.sendMessage();
        action = commun_FORWARD;
    }

    static void handleSpeed()
    {
        esp_now_message.sendMessage();
        speed = html_server.getVal();
    }

    static void handleBackward()
    {
        esp_now_message.sendMessage();
        action = commun_BACKWARD;
    }

    static void handleLeft()
    {
        esp_now_message.sendMessage();
        action = commun_LEFT;
    }

    static void handleRight()
    {
        esp_now_message.sendMessage();
        action = commun_RIGHT;
    }

    static void handleStop()
    {
        esp_now_message.sendMessage();
        action = commun_STOP;
    }

    static void handleTurnRate()
    {
        esp_now_message.sendMessage();
        turnRate = html_server.getVal();
    }
};

WiFiServer web_commun::wifi_server(80);
HTML510Server web_commun::html_server(80);
commun_Mode web_commun::mode = commun_NOTHING;
commun_Actions web_commun::action = commun_STOP;
int web_commun::speed = 0;
int web_commun::turnRate = 50;

class UDP_broadcast
{ // use mamber function sendXY to broadcast
public:
    int signalPin1 = 8; // GPIO pin receiving signal from Vive circuit
    int signalPin2 = 18;

    WiFiUDP UDPServer;
    const char *ssid = "TP-Link_FD24";
    const char *password = "65512111";
    IPAddress target; // broadcast mode is 255
    IPAddress myIP;   // change our IP
    char udpBuffer[14];
    int GroupNumber = 30;

    UDP_broadcast()
    {
        Serial.begin(115200);

        target = (192, 168, 1, 255);
        myIP = (192, 168, 1, 57);

        WiFi.begin(ssid, password);

        WiFi.config(myIP,                         // device IP address
                    IPAddress(192, 168, 1, 1),    // gateway (not used)
                    IPAddress(255, 255, 255, 0)); // netmask

        UDPServer.begin(2808); // 2808 arbitrary UDP port#
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.printf("WiFi connected to %s", ssid);
        Serial.print("Sending messages from ");
        Serial.print(myIP);
        Serial.print(" to ");
        Serial.println(target);
    }

    ~UDP_broadcast() {}

    void sendXY(int x, int y)
    { // use in main function to broadcast XY
        std::sprintf(udpBuffer, "%02d,%04d,%04d", GroupNumber, x, y);
        UDPServer.beginPacket(target, 2808); // send to UDPport 2808
        UDPServer.printf("%s", udpBuffer);
        UDPServer.endPacket();
        Serial.println(udpBuffer);
    }
};

class Serial_commun {
public:
  String message;
  SoftwareSerial softSerial;

  Serial_commun(int RX_PIN, int TX_PIN)
    : softSerial(RX_PIN, TX_PIN) {
    Serial.begin(115200);
    softSerial.begin(9600);
  }

  ~Serial_commun() {}

  void read() {
    if (softSerial.available()) {
      message = softSerial.readString();
      Serial.println("Received message on ESP32 2: " + message);
    }
  }

  void write(String wr_message) {
    Serial.println("Sending message: " + wr_message);
    softSerial.println(wr_message);
  }
};

#endif