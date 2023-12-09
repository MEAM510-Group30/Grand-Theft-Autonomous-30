/*
    Description: In this file, we define functions to communicate betweeen esp_c3 and esp_s2
*/

#ifndef C3_I2C_H
#define C3_I2C_H

#include <SoftwareSerial.h>
// Include any necessary headers here

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
        Serial.begin(115200);
        WiFi.mode(WIFI_STA);
        Serial.print("Sending MAC: ");
        Serial.println(WiFi.macAddress());

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
class Serial_commun
{ // use this class in main to communicate between two boards. to initialize, give tx and rx
public:
    String message;
    SoftwareSerial softSerial;
    esp_now esp_now_message;

    Serial_commun(int RX_PIN, int TX_PIN)
        : softSerial(RX_PIN, TX_PIN)
    {
        Serial.begin(115200);
        softSerial.begin(9600);
        esp_now_message.initialize();
    }

    ~Serial_commun() {}

    void read()
    {
        if (softSerial.available())
        {
            message = softSerial.readString();
            Serial.println("Received message on ESP32 2: " + message);
        }
        if (message == "broadcast")
        {
            esp_now_message.sendMessage();
        }
    }

    void write(String wr_message)
    {
        Serial.println("Sending message: " + wr_message);
        softSerial.println(wr_message);
    }
};

#endif