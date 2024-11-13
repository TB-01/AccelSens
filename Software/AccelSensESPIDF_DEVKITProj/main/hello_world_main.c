
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "freertos/timers.h"
// #include "driver/i2c_master.h"           //the newer I2C driver 
#include "esp_system.h"
#include <math.h>
#include "esp_log.h"
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h> //Requires by memset
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>








#define I2C_MASTER_SCL_IO    22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21               /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM       I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   400000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master do not need buffer */
#define MPU9250_SENSOR_ADDR  0x68             /*!< Slave address of the MPU9250 sensor */
#define WRITE_BIT            I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT             I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN         0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS        0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL              0x0              /*!< I2C ack value */
#define NACK_VAL             0x1              /*!< I2C nack value */



// MPU9250 Registers 
#define MPU9250_REG_PWR_MGMT_1   0x6B
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_GYRO_XOUT_H  0x43

#define GYRO_RANGE_250_DPS          0
#define GYRO_RANGE_500_DPS          1
#define GYRO_RANGE_1000_DPS         2
#define GYRO_RANGE_2000_DPS         3

#define ACCELEROMETER_RANGE_2G      0      
#define ACCELEROMETER_RANGE_4G      1      
#define ACCELEROMETER_RANGE_8G      2      
#define ACCELEROMETER_RANGE_16G     3

#define LED_PIN 2

// Sensor register addresses within the MPU9250 and the AK8963 sensor (which is the magnetometer component of the MPU9250 system).
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B

static const float CONFIG_ACCELEROMETER_RANGE       = ACCELEROMETER_RANGE_2G;


//used during magnetometer calibration to assure sufficient range coverage

//=============================================================
//================== CONFIGURATION CONSTANTS ==================  
//=============================================================


static esp_err_t mpu9250_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t mpu9250_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    // Start a command link transaction for the address setting phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);  // Send the device address with the write option
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);                     // Send the register address
    i2c_master_stop(cmd);                                                   // Stop after writing the register address
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    if (ret != ESP_OK) {
        return ret;  // Return error if address setting phase failed
    }

    // Start another command link transaction for the reading phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | READ_BIT, ACK_CHECK_EN);  // Send the device address with the read option
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);                       // Read data bytes with ACK
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);                    // Read the last byte with NACK
    i2c_master_stop(cmd);                                                   // Stop after reading the data
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    return ret;  // Return the result of the reading phase
}

/// @brief 
/// @param  
/// @return 
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t set_accel_range(uint8_t fs_sel) {
    if (fs_sel > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = mpu9250_write_byte(MPU9250_SENSOR_ADDR, ACCEL_CONFIG, fs_sel << 3);
    if (ret != ESP_OK) {
        printf("Failed to set_accel_range: %s\n", esp_err_to_name(ret));
    }
    else
    {
        printf("Accel Range succesfully set");
    }
    return ret;
}


float get_accel_sensitivity() {
    uint8_t accel_sensitivity_setting;
    mpu9250_read_bytes(MPU9250_SENSOR_ADDR, ACCEL_CONFIG, &accel_sensitivity_setting, 1);

    switch (accel_sensitivity_setting & 0x18) {
        case 0x00: return 2.0 / 32768.0;
        case 0x08: return 4.0 / 32768.0;
        case 0x10: return 8.0 / 32768.0;
        case 0x18: return 16.0 / 32768.0;
        default:   return 2.0 / 32768.0;  // Default sensitivity
    }
}



float get_gyro_sensitivity() {
    uint8_t gyro_sensitivity_setting;
    mpu9250_read_bytes(MPU9250_SENSOR_ADDR, GYRO_CONFIG, &gyro_sensitivity_setting, 1);

    switch (gyro_sensitivity_setting & 0x18) {
        case 0x00:  // ±250 °/s
            return 250.0 / 32768.0;
        case 0x08:  // ±500 °/s
            return 500.0 / 32768.0;
        case 0x10:  // ±1000 °/s
            return 1000.0 / 32768.0;
        case 0x18:  // ±2000 °/s
            return 2000.0 / 32768.0;
        default:
            return 250.0 / 32768.0;  // Default sensitivity
    }
}




void mpu9250_task() {
    uint8_t sensor_data[14];    // Buffer for accelerometer and gyroscope
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    float MYDATATIMER = 0;
    float accel_sensitivity = get_accel_sensitivity();
    float gyro_sensitivity = get_gyro_sensitivity();

    while (1) {
        // Read accelerometer, gyroscope, and magnetometer data
        mpu9250_read_bytes(MPU9250_SENSOR_ADDR, MPU9250_REG_ACCEL_XOUT_H, sensor_data, 14);
        //printf("%ld", i);
        // Parse and calibrate data from sensors
        accel_x = ((sensor_data[0] << 8) | sensor_data[1]);
        accel_y = ((sensor_data[2] << 8) | sensor_data[3]);
        accel_z = ((sensor_data[4] << 8) | sensor_data[5]);
        gyro_x = ((sensor_data[8] << 8) | sensor_data[9]);
        gyro_y = ((sensor_data[10] << 8) | sensor_data[11]);
        gyro_z = ((sensor_data[12] << 8) | sensor_data[13]);


        // Convert raw data to logical values with units
        float accel_x_g = accel_x * accel_sensitivity;
        float accel_y_g = accel_y * accel_sensitivity;
        float accel_z_g = accel_z * accel_sensitivity;
        float gyro_x_dps = gyro_x * gyro_sensitivity;
        float gyro_y_dps = gyro_y * gyro_sensitivity;
        float gyro_z_dps = gyro_z * gyro_sensitivity;

        // Calculate roll and pitch
        float roll = atan2(accel_y_g, accel_z_g) * (180.0 / M_PI);
        float pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * (180.0 / M_PI);

        MYDATATIMER = MYDATATIMER +(portTICK_PERIOD_MS) ;

        printf("%7.3f,%7.2f,%7.2f,%7.2f\n",
               MYDATATIMER, accel_x_g, accel_y_g, accel_z_g);

        //vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}







void app_main()
{
    

    // Initialize the I2C bus for communication
    i2c_master_init();

    set_accel_range(CONFIG_ACCELEROMETER_RANGE);  // 0b00 corresponds to ±2g

    //starten der Beschleunigungsmessung mit durch auslesen des Busses
    mpu9250_task();


}


