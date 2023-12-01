// I'm still thinking about whether this function should be made a seperete header file or just within the ino file.

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <iostream>
#include <cstdint>
#include "website.h"
#include "html510.h"
#include "s2_vive510.h"
class web_commun
{
    Actions action;
    Behavior behavior;

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
        action.speed = html_server.getVal();
    }

    void handleTurnRate()
    {
        action.turnRate = html_server.getVal();
    }
}

class UDP_broadcast
{
    int signalPin 4; // GPIO pin receiving signal from Vive circuit
    Vive510 vive1(signalPin);

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

    void sendXY()
    {
        if (vive1.status() == VIVE_LOCKEDON)
        {
            std::sprintf(udpBuffer, "%02d,%04d,%04d", teamNumber, vive1.xCoord(), vive1.yCoord());
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

#endif