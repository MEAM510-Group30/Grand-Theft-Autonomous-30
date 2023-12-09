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
{                 // remember to initialize; use member function sendMessage() to send group number
#define CHANNEL 1 // channel can be 1 to 14, channel 0 means current channel.

public:
    int groupNumber = 30;
    String message = String(groupNumber);
    esp_now_peer_info_t peer1;
    uint8_t MAC_RECV[6] = {0x68, 0x67, 0x25, 0x82, 0x8C, 0xCC}; // receiver MAC address

    esp_now()
    {
        memcpy(peer1.peer_addr, MAC_RECV, 6);
        peer1.channel = 0; // Set your channel here
        peer1.encrypt = false;
    }

    ~esp_now() {}

    void initialize()
    {
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("init failed");
            ESP.restart();
        }
        else
            Serial.println("init succeed");
        // esp_now_register_send_cb(OnDataSent); // optional if you want ack interrupt

        if (esp_now_add_peer(&peer1) != ESP_OK)
        {
            Serial.println("Pair failed"); // ERROR  should not happen
        }
        else
            Serial.println("Pair succeed");

        esp_now_register_send_cb(OnDataSent);
    }

    void WiFi_initial()
    {
        Serial.begin(115200);
        WiFi.mode(WIFI_STA);
        Serial.print("Sending MAC: ");
        Serial.println(WiFi.macAddress());
    }

    static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if (status == ESP_NOW_SEND_SUCCESS)
            Serial.println("Success ");
        else
            Serial.println("Fail ");
    }

    void sendMessage()
    { // use in main function to send group number every time communicate with web       // message is the group number
        if (esp_now_send(MAC_RECV, (uint8_t *)message.c_str(), message.length()) == ESP_OK)
            Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n", message, peer1.peer_addr[0], peer1.peer_addr[1], peer1.peer_addr[2], peer1.peer_addr[3], peer1.peer_addr[4], peer1.peer_addr[5]);
        else
        {
            esp_err_t sendResult = esp_now_send(peer1.peer_addr, reinterpret_cast<const uint8_t *>(message.c_str()), message.length());
            Serial.printf("Send failed, error code: %d\n", sendResult);
        }
    }
};

class web_commun
{   // in main function, initialize a web_commun variable to attach handlers
    // visit commun_Mode, Action, speed, turnRate in main function

public:
    static esp_now esp_now_message;
    static commun_Mode mode;
    static commun_Actions action;
    static int speed;
    static int turnRate;
    static WiFiServer wifi_server;
    static HTML510Server html_server;
    const char *ssid = "group30"; // set my router ID
    const char *pwd = "12345678"; // set router password
    static SemaphoreHandle_t espNowSemaphore;

    web_commun()
    {
        espNowSemaphore = xSemaphoreCreateMutex();
    }

    void initial(IPAddress local_IP, const char *ssid = "group30", const char *pwd = "12345678")
    {
        // our own IP address is local_IP above
        IPAddress gateway_IP(192, 168, 1, 1);
        IPAddress subnet_IP(255, 255, 255, 0);

        Serial.begin(115200);
        WiFi.mode(WIFI_MODE_AP); // wifi in ap mode, no router
        Serial.print("Access point ");
        Serial.println(ssid);
        WiFi.softAP(ssid, pwd);
        WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP);
        Serial.print("AP IP address ");
        Serial.println(local_IP);
        // wifi_server.begin();
        // IPAddress softAP_IP = WiFi.softAPIP();
        esp_now_message.initialize();

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

        espNowSemaphore = xSemaphoreCreateMutex();
    }

    ~web_commun() {}

    static void handleRoot()
    {
        html_server.sendhtml(body);
    }

    static void handleTrophy()
    {
        // esp_now_message.sendMessage();
        mode = commun_TROPHY;
        Serial.print("Trophy");
    }

    static void handleManual()
    {
        // esp_now_message.sendMessage();
        mode = commun_MANUAL;
        Serial.print("Manual");
    }

    static void handleWall()
    {
        // esp_now_message.sendMessage();
        mode = commun_WALL;
        Serial.print("Wall");
    }

    static void handleCar()
    {
        // esp_now_message.sendMessage();
        mode = commun_PUSH;
        Serial.print("Car");
    }

    static void handleForward()
    {
        // esp_now_message.sendMessage();
        action = commun_FORWARD;
        Serial.print("Forward");
    }

    static void handleSpeed()
    {
        // esp_now_message.sendMessage();
        speed = html_server.getVal();
        Serial.printf("set speed %d", speed);
    }

    static void handleBackward()
    {
        // esp_now_message.sendMessage();
        action = commun_BACKWARD;
        Serial.print("Backward");
    }

    static void handleLeft()
    {
        // esp_now_message.sendMessage();
        action = commun_LEFT;
        Serial.print("Left");
    }

    static void handleRight()
    {
        // esp_now_message.sendMessage();
        action = commun_RIGHT;
        Serial.print("Right");
    }

    static void handleStop()
    {
        // esp_now_message.sendMessage();
        action = commun_STOP;
        Serial.print("Stop");
    }

    static void handleTurnRate()
    {
        // esp_now_message.sendMessage();
        turnRate = html_server.getVal();
        Serial.printf("set TurnRate %d", turnRate);
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
    IPAddress gateway_IP = IPAddress(192, 168, 4, 1);
    IPAddress subnet_IP = IPAddress(255, 255, 255, 0);
    WiFiServer server;
    char udpBuffer[20];
    char packetBuffer[20];
    int GroupNumber = 30;
    const int UDP_PACKET_SIZE = 20;

    UDP_broadcast(IPAddress IP) : server(80)
    {
        target = IPAddress(192, 168, 1, 255);
        myIP = IP;
    }

    ~UDP_broadcast() {}

    void initialBroadcast()
    {
        Serial.begin(115200);
        WiFi.mode(WIFI_AP); // wifi in ap mode, no router
        WiFi.softAPConfig(myIP, gateway_IP, subnet_IP);
        WiFi.softAP(ssid, password);

        IPAddress softAP_IP = WiFi.softAPIP();
        Serial.print("\n AP IP address: HTML//");
        Serial.print(softAP_IP);
        Serial.print("\n SSID: ");
        Serial.print(ssid);
        Serial.print("\n Password: ");
        Serial.print(password);

        server.begin();

        UDPServer.begin(2808); // 2808 arbitrary UDP port#
    }

    void sendXY(int x, int y)
    { // use in main function to broadcast XY
        std::sprintf(udpBuffer, "%02d,%04d,%04d", GroupNumber, x, y);
        UDPServer.beginPacket(target, 2808); // send to UDPport 2808
        UDPServer.printf("%s", udpBuffer);
        UDPServer.endPacket();
        Serial.println(udpBuffer);
    }

    void handleUDPServer()
    {
        int cb = UDPServer.parsePacket();
        if (cb)
        {
            UDPServer.read(packetBuffer, UDP_PACKET_SIZE - 1);
            Serial.println(packetBuffer);
            return;
        }
        Serial.println("no data received");
    }
};

class Serial_commun
{ // use this class in main to communicate between two boards. to initialize, give tx and rx
public:
    String message;
    SoftwareSerial softSerial;

    Serial_commun(int RX_PIN, int TX_PIN)
        : softSerial(RX_PIN, TX_PIN)
    {
        Serial.begin(115200);
        softSerial.begin(9600);
    }

    ~Serial_commun() {}

    void read()
    {
        if (softSerial.available())
        {
            message = softSerial.readString();
            Serial.println("Received message on ESP32 2: " + message);
        }
    }

    void write(String wr_message)
    {
        Serial.println("Sending message: " + wr_message);
        softSerial.println(wr_message);
    }
};

#endif