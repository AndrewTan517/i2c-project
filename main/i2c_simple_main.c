/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU6050 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu6050.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_ACCEL_XOUT_H_REG_ADDR       0x3B
#define MPU6050_ACCEL_XOUT_L_REG_ADDR       0x3C
#define MPU6050_ACCEL_YOUT_H_REG_ADDR       0x3D
#define MPU6050_ACCEL_YOUT_L_REG_ADDR       0x3E
#define MPU6050_ACCEL_ZOUT_H_REG_ADDR       0x3F
#define MPU6050_ACCEL_ZOUT_L_REG_ADDR       0x40
#define MPU6050_TEMP_OUT_H_REG_ADDR         0x41
#define MPU6050_TEMP_OUT_L_REG_ADDR         0x42
#define MPU6050_GYRO_XOUT_H_REG_ADDR        0x43
#define MPU6050_GYRO_XOUT_L_REG_ADDR        0x44
#define MPU6050_GYRO_YOUT_H_REG_ADDR        0x45
#define MPU6050_GYRO_YOUT_L_REG_ADDR        0x46
#define MPU6050_GYRO_ZOUT_H_REG_ADDR        0x47
#define MPU6050_GYRO_ZOUT_L_REG_ADDR        0x48

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
// static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
// static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
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

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    // uint8_t data[14];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");    

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
    // ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    // ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Reset the MPU6050 before reading values */
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));
    // vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for the reset to complete

    /* Take the MPU6050 out of sleep mode */
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0));

    /* Demonstrate reading the accelerometer, gyroscope and temperature values from the MPU6050 */
    // ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_XOUT_H_REG_ADDR, data, 14));
    // int16_t acce_x = (data[0] << 8) | data[1];
    // int16_t acce_y = (data[2] << 8) | data[3];
    // int16_t acce_z = (data[4] << 8) | data[5];
    // int16_t temp = (data[6] << 8) | data[7];
    // int16_t gyro_x = (data[8] << 8) | data[9];
    // int16_t gyro_y = (data[10] << 8) | data[11];
    // int16_t gyro_z = (data[12] << 8) | data[13];

    // ESP_LOGI(TAG, "ACCE_X = %d", acce_x);
    // ESP_LOGI(TAG, "ACCE_Y = %d", acce_y);
    // ESP_LOGI(TAG, "ACCE_Z = %d", acce_z);
    // ESP_LOGI(TAG, "TEMP = %d", temp);
    // ESP_LOGI(TAG, "GYRO_X = %d", gyro_x);
    // ESP_LOGI(TAG, "GYRO_Y = %d", gyro_y);
    // ESP_LOGI(TAG, "GYRO_Z = %d", gyro_z);

    /* Initialize the MPU6050 sensor */
    mpu6050_handle_t sensor = mpu6050_create(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR);
    mpu6050_acce_fs_t acce_fs = ACCE_FS_8G;
    mpu6050_gyro_fs_t gyro_fs = GYRO_FS_1000DPS;
    ESP_ERROR_CHECK(mpu6050_config(sensor, acce_fs, gyro_fs));

    ESP_ERROR_CHECK(mpu6050_wake_up(sensor));

    while (1)
    {
        mpu6050_acce_value_t acce_value[3];      
        ESP_ERROR_CHECK(mpu6050_get_acce(sensor, acce_value));
        ESP_LOGI(TAG, "ACCE_X = %f", acce_value->acce_x);
        ESP_LOGI(TAG, "ACCE_Y = %f", acce_value->acce_y);
        ESP_LOGI(TAG, "ACCE_Z = %f", acce_value->acce_z);

        mpu6050_gyro_value_t gyro_value[3];
        ESP_ERROR_CHECK(mpu6050_get_gyro(sensor, gyro_value));
        ESP_LOGI(TAG, "GYRO_X = %f", gyro_value->gyro_x);
        ESP_LOGI(TAG, "GYRO_Y = %f", gyro_value->gyro_y);
        ESP_LOGI(TAG, "GYRO_Z = %f", gyro_value->gyro_z);

        mpu6050_temp_value_t temp_value[1];
        ESP_ERROR_CHECK(mpu6050_get_temp(sensor, temp_value));
        ESP_LOGI(TAG, "TEMP = %f", temp_value->temp);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(mpu6050_sleep(sensor));

    mpu6050_delete(sensor);

    /* Demonstrate writing by reseting the MPU6050 */
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
