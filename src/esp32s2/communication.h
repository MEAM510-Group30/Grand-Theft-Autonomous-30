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

    void handleRoot()
    {
        html_server.sendhtml(website);
    }

    void handleTrophy()
    {
        esp_now_message.sendMessage();
        behavior.trophyMoving();
    }

    void handleManual()
    {
        esp_now_message.sendMessage();
        behavior.fullyManual();
    }

    void handleWall()
    {
        esp_now_message.sendMessage();
        behavior.wallFollowing();
    }

    void handleCar()
    {
        behavior.carPushing();
        esp_now_message.sendMessage();
    }

    void handleForward()
    {
        esp_now_message.sendMessage();
        action.moveForward(action.speed);
    }

    void handleSpeed()
    {
        esp_now_message.sendMessage();
        action.speed = html_server.getVal();
    }

    void handleBackward()
    {
        esp_now_message.sendMessage();
        action.moveBackward(action.speed);
    }

    void handleLeft()
    {
        esp_now_message.sendMessage();
        action.turnLeft(action.speed, action.turnRate);
    }

    void handleRight()
    {
        esp_now_message.sendMessage();
        action.turnRight(action.speed, action.turnRate);
    }

    void handleStop()
    {
        esp_now_message.sendMessage();
        action.stop();
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
        message = 30; // message is the group number
        if (esp_now_send(peer1.peer_addr, message, sizeof(message)) == ESP_OK)
            Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n", message, peer1.peer_addr[0], peer1.peer_addr[1], peer1.peer_addr[2], peer1.peer_addr[3], peer1.peer_addr[4], peer1.peer_addr[5]);
        else
            Serial.println("Send failed");
    }
}

class I2C_commun()
{
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define DATA_LENGTH 128   /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 10 /*!< Data length for r/w test, [0,DATA_LENGTH] */

#define I2C_MASTER_SCL_IO (gpio_num_t)4 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO (gpio_num_t)5 /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x28        /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK     /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_NACK   /*!< I2C nack value */

    I2C_commun()
    {
        Serial.begin(115200); // put your setup code here, to run once:
        i2c_master_init();
    }

    ~I2C_commun(){}

    static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t * data_rd, size_t nsize)
    {
        if (nsize == 0)
        {
            return ESP_OK;
        }
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
        if (nsize > 1)
        {
            i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
        }
        i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t * data_wr, size_t nsize)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    /**
     * @brief i2c master initialization
     */
    static esp_err_t i2c_master_init()
    {
        i2c_port_t i2c_master_port = I2C_NUM_0;
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        i2c_param_config(i2c_master_port, &conf);
        return i2c_driver_install(i2c_master_port, conf.mode,
                                  I2C_MASTER_RX_BUF_DISABLE,
                                  I2C_MASTER_TX_BUF_DISABLE, 0);
    }
}

#endif