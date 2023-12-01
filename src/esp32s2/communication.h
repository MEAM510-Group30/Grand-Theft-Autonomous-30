// I'm still thinking about whether this function should be made a seperete header file or just within the ino file.

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <iostream>
#include <cstdint>
#include <esp_now.h>
#include "website.h"
#include "html510.h"
#include "s2_vive510.h"
class web_commun
{
    Actions action;
    Behavior behavior;
    esp_now esp_now_message;

    web_commun(IPAddress local_IP, const char *ssid = "Furina", const char *pwd = "Furinaaa") :
    {
        // our own IP address is local_IP above
        IPAddress gateway_IP(192, 168, 1, 1);
        IPAddress subnet_IP(255, 255, 255, 0);
        WiFiServer wifi_server(8080);
        HTML510Server html_server(80);

        WiFi.mode(WIFI_MODE_AP); // wifi in ap mode, no router
        WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP);
        WiFi.softAP(ssid, pwd);
        wifi_server.begin();
        IPAddress softAP_IP = WiFi.softAPIP();

        html_server.begin();
        html_server.attachHandler("/", handleRoot);
        // html_server.attachHandler("/autopilot_on", handleAutopilotOn);
        // html_server.attachHandler("/autopilot_off", handleAutopilotOff);
        html_server.attachHandler("/tro", behavior.trophyMoving());
        html_server.attachHandler("/man", behavior.fullyManual());
        html_server.attachHandler("/wall", behavior.wallFollowing());
        html_server.attachHandler("/car", carPushing());
        html_server.attachHandler("/F", action.moveForward(action.speed));
        html_server.attachHandler("/B", action.moveBackward(action.speed));
        html_server.attachHandler("/L", action.turnLeft(action.speed, action.turnRate));
        html_server.attachHandler("/R", action.turnRight(action.speed, action.turnRate));
        html_server.attachHandler("/O", action.stop());
        html_server.attachHandler("/speed_slider=", handleSpeed);
        html_server.attachHandler("/turn_rate_slider=", handleTurnRate);
    }

    ~web_commun() {}

    void handleRoot()
    {
        html_server.sendhtml(website);
    }

    void handleSpeed()
    {
        esp_now_message.sendMessage();
        action.speed = html_server.getVal();
    }

    void handleTurnRate()
    {
        esp_now_message.sendMessage();
        action.turnRate = html_server.getVal();
    }
}

class UDP_broadcast
{
    int signalPin1 8; // GPIO pin receiving signal from Vive circuit
    int signalPin2 18;
    int xLocation = 0; // x location of the center of the mobile base, which is the average of 2 vives
    int yLocation = 0; // y location of the center of the mobile base, which is the average of 2 vives
    Vive510 vive1(signalPin1);
    Vive510 vive2(signalPin2);

    WiFiUDP UDPServer;
    const char *ssid = "TP-Link_FD24";
    const char *password = "65512111";
    IPAddress target(192, 168, 1, 255); // broadcast mode is 255
    IPAddress myIP(192, 168, 1, 57);    // change our IP
    char udpBuffer[14];
    int GroupNumber = 30;

    UDP_broadcast()
    {
        Serial.begin(115200);

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

    void sendXY()
    {
        if (vive1.status() == VIVE_LOCKEDON && vive2.status() == VIVE_LOCKEDON)
        {
            xLocation = 0.5 * (vive1.xCoord() + vive2.xCoord());
            yLocation = 0.5 * (vive1.yCoord() + vive2.yCoord());

            std::sprintf(udpBuffer, "%02d,%04d,%04d", teamNumber, xLocation, yLocation);
            UDPServer.beginPacket(target, 2808); // send to UDPport 2808
            UDPServer.printf("%s", udpBuffer);
            UDPServer.endPacket();
            Serial.println(udpBuffer);
            Serial.printf("X %d, Y %d\n", vive1.xCoord(), vive1.yCoord());
        }
        else
            vive1.sync(15); // try to resync 15 times (nonblocking);
    }
}

class esp_now()
{
#define CHANNEL 1                                     // channel can be 1 to 14, channel 0 means current channel.
#define MAC_RECV {0x84, 0xF7, 0x03, 0xA8, 0xBE, 0x30} // receiver MAC address
    uint8_t message;

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

        esp_now_register_send_cb(OnDataSent); // optional if you want ack interrupt

        if (esp_now_add_peer(&peer1) != ESP_OK)
        {
            Serial.println("Pair failed"); // ERROR  should not happen
        }
    }

    ~esp_now() {}

    esp_now_peer_info_t peer1 =
        {
            .peer_addr = MAC_RECV,
            .channel = CHANNEL,
            .encrypt = false,
        };

    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if (status == ESP_NOW_SEND_SUCCESS)
            Serial.println("Success ");
        else
            Serial.println("Fail ");
    }

    void sendMessage()
    {
        message = 30;   //message is the group number
        if (esp_now_send(peer1.peer_addr, message, sizeof(message)) == ESP_OK)
            Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n", message, peer1.peer_addr[0], peer1.peer_addr[1], peer1.peer_addr[2], peer1.peer_addr[3], peer1.peer_addr[4], peer1.peer_addr[5]);
        else
            Serial.println("Send failed");
    }
}

#endif