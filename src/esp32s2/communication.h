// I'm still thinking about whether this function should be made a seperete header file or just within the ino file.

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "website.h"
#include "html510.h"
class web_commun
{
    Actions action;
    int speed = 0;

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
        html_server.attachHandler("/autopilot_on", handleAutopilotOn);
        html_server.attachHandler("/autopilot_off", handleAutopilotOff);
        html_server.attachHandler("/speed_slider=", handleSpeed);
        html_server.attachHandler("/turn_rate_slider=", handleTurnRate);
        html_server.attachHandler("/F", action.moveForward(speed));
        html_server.attachHandler("/B", action.moveBackward(speed));
        html_server.attachHandler("/L", action.turnLeft(speed, turnRate));
        html_server.attachHandler("/R", action.turnRight(speed, turnRate));
        html_server.attachHandler("/O", action.stop());
        html_server.attachHandler("/S", action.stop());
        html_server.attachHandler("/+", handleSpeedUp);
        html_server.attachHandler("/-", handleSlowDown);
    }

    ~web_commun() {}

    void handleSpeedUp(){
        action.speed += 500;
    }

    void handleSlowDown(){
        action.speed -= 500;
    }
}

#endif