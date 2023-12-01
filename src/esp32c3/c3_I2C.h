/*
    Description: In this file, we define functions to communicate betweeen esp_c3 and esp_s2
*/

#ifndef C3_I2C_H
#define C3_I2C_H

// Include any necessary headers here
class I2C_commun
{
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define DATA_LENGTH 128   /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 22 /*!< Data length for r/w test, [0,DATA_LENGTH] */

#define I2C_SLAVE_SCL_IO (gpio_num_t)4         /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO (gpio_num_t)5         /*!< gpio number for i2c slave data */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size */

#define ESP_SLAVE_ADDR 0x28 /*!< ESP32 slave address, you can set any 7bit value */

    I2C_commu()
    {
        Serial.begin(115200); // put your setup code here, to run once:
        i2c_slave_init();
    }

    ~I2C_commun(){}

    static esp_err_t i2c_slave_init()
    {
        i2c_port_t i2c_slave_port = I2C_NUM_0;
        i2c_config_t conf_slave;
        conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
        conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
        conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf_slave.mode = I2C_MODE_SLAVE;
        conf_slave.slave.addr_10bit_en = 0;
        conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
        i2c_param_config(i2c_slave_port, &conf_slave);
        return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                                  I2C_SLAVE_RX_BUF_LEN,
                                  I2C_SLAVE_TX_BUF_LEN, 0);
    }
}

#endif