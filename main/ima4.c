#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/i2c_master.h"
#include "esp_log.h" 

#define SDA_GPIO 21
#define SCL_GPIO 22
#define MPU_ADDR 0x68 // AD0 -> GND

static const char *TAG = "mpu6050";

void app_main(void) {
	// creating the bus
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0, 
		.sda_io_num = SDA_GPIO, 
		.scl_io_num = SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT, 
		.glitch_ignore_cnt = 7, // default value
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
	
	
	// checking if anyone is on the bus
	for (uint8_t addr = 1; addr < 127; addr++) {
		if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
			ESP_LOGI(TAG, "find device at 0x%02X (expected 0x68)", addr);
		}
	}
	
	
	// adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7, 
		.device_address = MPU_ADDR, 
		.scl_speed_hz = 400000, // 400kHz is max -> fast mode, 100kHz -> slow mode
	};
	i2c_master_dev_handle_t mpu;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mpu));
	
	
	// checking WHO_AM_I register
	uint8_t reg = 0x75;
	uint8_t who = 0;
	
	ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu, &reg, 1, &who, 1, 100));
	ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
	
	if (who != 0x68) {
	    ESP_LOGE(TAG, "unexpected WHO_AM_I");
	    return;
	}
	
	
	// waking up the mpu and set gyro clk source
	uint8_t wake [2] = {0x6B, 0x01};
	
	ESP_ERROR_CHECK(i2c_master_transmit(mpu, wake, 2, 100));
	vTaskDelay(pdMS_TO_TICKS(100)); // wait for sensor startup
	
	
	// Set accel range to +-2g explicitly
	uint8_t acc_cfg[2] = {0x1C, 0x00};
	ESP_ERROR_CHECK(i2c_master_transmit(mpu, acc_cfg, 2, 100));
	
	
	// get accell in loop
	while (1) {
		uint8_t start_reg = 0x3B;
		uint8_t r[6];
		
		esp_err_t err = i2c_master_transmit_receive(mpu, &start_reg, 1, r, 6, 100);
		
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
