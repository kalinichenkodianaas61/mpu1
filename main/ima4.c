#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/i2c_master.h"
#include "esp_log.h" 

// i2c4
#define SDA_GPIO 21
#define SCL_GPIO 22
#define MPU_ADDR 0x68 // AD0 -> GND
#define MPU6050_SCL_SPEED_HZ 100000 // 100kHz -> slow mode, 400kHz -> fast mode

// registers
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_ACCEL_CFG_REG 0x1C
#define MPU6050_AFS_SEL 0x00 // 0 -> +-2g
#define MPU6050_PWR_MGMT1_REG 0x6B
#define MPU6050_PWR_MGMT1_ARG 0x01 // enable PLL with X axis gyroscope reference

// delays
#define MPU6050_DELAY_WAKE_UP_MS 100 // wait after sensor wakes up


// i2c transaction timeout
#define MPU6050_XFER_TIMEOUT_MS 100

static const char *TAG = "mpu6050";

static esp_err_t mpu6050_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;
	
	// (1) creating the bus
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0, 
		.sda_io_num = SDA_GPIO, 
		.scl_io_num = SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT, 
		.glitch_ignore_cnt = 7, // default value
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus;
	err = i2c_new_master_bus(&bus_cfg, &bus);
	if (err != ESP_OK) {
		return err;
	}
	
	
	// (2) adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7, 
		.device_address = MPU_ADDR, 
		.scl_speed_hz = MPU6050_SCL_SPEED_HZ,
	};
	err = i2c_master_bus_add_device(bus, &dev_cfg, dev);
	if (err != ESP_OK) {
		return err;
	}
	
	
	// (3) checking WHO_AM_I register
	uint8_t who_am_i_reg = MPU6050_WHO_AM_I_REG;
	uint8_t who = 0;

	err = i2c_master_transmit_receive(*dev, &who_am_i_reg, 1, &who, 1, MPU6050_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	
	if (who == MPU_ADDR) {
		ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
	} else 	{
	    ESP_LOGW(TAG, "unexpected WHO_AM_I: 0x%02X", who);
	}
	
	// (4) waking up the mpu and set gyro clk source
	uint8_t wake [2] = {MPU6050_PWR_MGMT1_REG, MPU6050_PWR_MGMT1_ARG};

	err = i2c_master_transmit(*dev, wake, 2, MPU6050_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	vTaskDelay(pdMS_TO_TICKS(MPU6050_DELAY_WAKE_UP_MS)); // wait for sensor startup


	// (5) set accel range to +-2g explicitly
	uint8_t acc_cfg[2] = {MPU6050_ACCEL_CFG_REG, MPU6050_AFS_SEL};
	err = i2c_master_transmit(*dev, acc_cfg, 2, MPU6050_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	
	return ESP_OK;
	
}

void app_main(void) {
	i2c_master_dev_handle_t mpu1;
	ESP_ERROR_CHECK(mpu6050_init(&mpu1));
	
	// get accell in loop
	while (1) {
		uint8_t start_reg = 0x3B;
		uint8_t r[6];
		
		esp_err_t err = i2c_master_transmit_receive(mpu1, &start_reg, 1, r, 6, 100);
		
		if (err == ESP_OK) {
			int16_t rawX = (int16_t) ((r[0] << 8) | r[1]);
			int16_t rawY = (int16_t) ((r[2] << 8) | r[3]);
			int16_t rawZ = (int16_t) ((r[4] << 8) | r[5]);
			
			ESP_LOGI(TAG, " raw: %6d %6d %6d | g: %+7.2f %+7.2f %+7.2f", 
					rawX, rawY, rawZ,
					rawX / 16384.0f, rawY / 16384.0f, rawZ / 16384.0f);
		} else {
			ESP_LOGE(TAG, "accel read failed: %s", esp_err_to_name(err));
		}
		
		vTaskDelay(pdMS_TO_TICKS(500));
	}

}
