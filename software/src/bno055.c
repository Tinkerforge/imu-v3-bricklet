/* imu-v3-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * bno055.c: Driver for BNO055
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "bno055.h"

#include "configs/config_bno055.h"

#include "communication.h"

#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

BNO055 bno055;
CoopTask bno055_task;


void bno055_init_i2c(void) {
	bno055.i2c_fifo.baudrate         = BNO055_I2C_BAUDRATE;
	bno055.i2c_fifo.address          = BNO055_I2C_ADDRESS;
	bno055.i2c_fifo.i2c              = BNO055_I2C;

	bno055.i2c_fifo.scl_port         = BNO055_SCL_PORT;
	bno055.i2c_fifo.scl_pin          = BNO055_SCL_PIN;
	bno055.i2c_fifo.scl_mode         = BNO055_SCL_PIN_MODE;
	bno055.i2c_fifo.scl_input        = BNO055_SCL_INPUT;
	bno055.i2c_fifo.scl_source       = BNO055_SCL_SOURCE;
	bno055.i2c_fifo.scl_fifo_size    = BNO055_SCL_FIFO_SIZE;
	bno055.i2c_fifo.scl_fifo_pointer = BNO055_SCL_FIFO_POINTER;

	bno055.i2c_fifo.sda_port         = BNO055_SDA_PORT;
	bno055.i2c_fifo.sda_pin          = BNO055_SDA_PIN;
	bno055.i2c_fifo.sda_mode         = BNO055_SDA_PIN_MODE;
	bno055.i2c_fifo.sda_input        = BNO055_SDA_INPUT;
	bno055.i2c_fifo.sda_source       = BNO055_SDA_SOURCE;
	bno055.i2c_fifo.sda_fifo_size    = BNO055_SDA_FIFO_SIZE;
	bno055.i2c_fifo.sda_fifo_pointer = BNO055_SDA_FIFO_POINTER;

	i2c_fifo_init(&bno055.i2c_fifo);
}

void bno055_task_read_register(const uint8_t reg, uint8_t *data, const uint8_t length) {
	while(true) {
		uint32_t ret = i2c_fifo_coop_read_register(&bno055.i2c_fifo, reg, length, data);
		if(ret != 0) {
			loge("i2c_fifo_coop_read_register error: %d\n\r", ret);
			bno055_init_i2c();
			coop_task_sleep_ms(200);
			continue;
		}
		break;
	}
}

// By trial and error we found out that the BNO055 sometimes does not
// change the fusion mode. There is no error (NAK during I2C communication or similar).
// Because of this we check everytime after we write a register and just try again if it does't match.
void bno055_task_write_register_with_check(const uint8_t reg, const uint8_t *data, const uint8_t length, const uint32_t sleep) {
	while(true) {
		coop_task_sleep_ms(2);
		uint8_t data_check[32] = {0xFF, 0xFF, 0xFF, 0xFF};
		uint32_t ret = i2c_fifo_coop_write_register(&bno055.i2c_fifo, reg, length, data, true);
		if(ret != 0) {
			loge("i2c_fifo_coop_write_register error: %d\n\r", ret);
			bno055_init_i2c();
			coop_task_sleep_ms(200);
			continue;
		}
		if(sleep <= 1) {
			// Always sleep at least 2ms here, just to be sure.
			// We only write when we change a configuration anyway.
			coop_task_sleep_ms(2);
		} else {
			coop_task_sleep_ms(sleep);
		}

		bno055_task_read_register(reg, data_check, length);
		if(memcmp(data, data_check, length) != 0) {
			loge("Register %x not set: %x vs %x\n\r", reg, data[0], data_check[0]);
			coop_task_sleep_ms(20);
			continue;
		}

		break;
	}
}

void bno055_task_update_sensor_data(void) {
	switch(bno055.update_sensor_counter) {
		case 0:
			bno055.sensor_done_copied  = false;
			bno055.update_sensor_start = system_timer_get_ms();
			bno055_task_read_register(BNO055_REG_ACC_DATA_X_LSB,  (uint8_t *)&bno055.sensor_data.acc_x,              sizeof(int16_t)*3);
			break;
		case 1:
			bno055_task_read_register(BNO055_REG_MAG_DATA_X_LSB,  (uint8_t *)&bno055.sensor_data.mag_x,              sizeof(int16_t)*3);
			break;
		case 2:
			bno055_task_read_register(BNO055_REG_GYR_DATA_X_LSB,  (uint8_t *)&bno055.sensor_data.gyr_x,              sizeof(int16_t)*3);
			break;
		case 3:
			bno055_task_read_register(BNO055_REG_EUL_HEADING_LSB, (uint8_t *)&bno055.sensor_data.eul_heading,        sizeof(int16_t)*3);
			break;
		case 4:
			bno055_task_read_register(BNO055_REG_QUA_DATA_W_LSB,  (uint8_t *)&bno055.sensor_data.qua_w,              sizeof(int16_t)*4);
			break;
		case 5:
			bno055_task_read_register(BNO055_REG_LIA_DATA_X_LSB,  (uint8_t *)&bno055.sensor_data.lia_x,              sizeof(int16_t)*3);
			break;
		case 6:
			bno055_task_read_register(BNO055_REG_GRV_DATA_X_LSB,  (uint8_t *)&bno055.sensor_data.grv_x,              sizeof(int16_t)*3);
			break;
		case 7:
			bno055_task_read_register(BNO055_REG_TEMP,            (uint8_t *)&bno055.sensor_data.temperature,        sizeof(int8_t)*1);
			break;
		case 8:
			bno055_task_read_register(BNO055_REG_CALIB_STAT,      (uint8_t *)&bno055.sensor_data.calibration_status, sizeof(int8_t)*1);
			break;
		default: // should always be in case of 9
			if(!bno055.sensor_done_copied) {
				memcpy(&bno055.sensor_data_ready, &bno055.sensor_data, sizeof(SensorData));
				bno055.sensor_done_copied = true;
			}

			// Wait for 10ms after we are finished with reading of data.
			// The BMO055 has an update rate of 100Hz and there is no "data-ready" pin or similar.
			if(!system_timer_is_time_elapsed_ms(bno055.update_sensor_start, 10)) {
				return;
			}
	}

	bno055.update_sensor_counter++;
	switch(bno055.sensor_fusion_mode) {
		case IMU_V3_SENSOR_FUSION_OFF: { // 0-2 -> 7 -> 9 -> 0
			if(bno055.update_sensor_counter == 3) {
				bno055.update_sensor_counter = 7; // Read temperature after acc, mag and gyr data
			} else if(bno055.update_sensor_counter == 8) {
				bno055.update_sensor_counter = 9; // Go to end after reading temperature
			} else if(bno055.update_sensor_counter == 10) {
				bno055.update_sensor_counter = 0;
			}

			break;
		}

		case IMU_V3_SENSOR_FUSION_ON_WITHOUT_FAST_MAGNETOMETER_CALIBRATION: // fall-through
		case IMU_V3_SENSOR_FUSION_ON: {
			if(bno055.update_sensor_counter == 10) { // 0-9 -> 0
				bno055.update_sensor_counter = 0;
			}

			break;
		}

		case IMU_V3_SENSOR_FUSION_ON_WITHOUT_MAGNETOMETER: {
			if(bno055.update_sensor_counter == 1) { // 0 -> 2-9 -> 0
				bno055.update_sensor_counter = 2;
			}
			if(bno055.update_sensor_counter == 10) {
				bno055.update_sensor_counter = 0;
			}

			break;
		}
	}
}

void bno055_task_update_sensor_fusion_mode(const bool already_in_config_mode) {
	uint8_t opr_mode = 0b000;
	if(!already_in_config_mode) {
		// First we go to config mode
		bno055_task_write_register_with_check(BNO055_REG_OPR_MODE, &opr_mode, 1, 20);
	}

	// If sensor fusion is turned off, we zero all of the now non-updated data
	switch(bno055.sensor_fusion_mode) {
		case IMU_V3_SENSOR_FUSION_OFF: {
			bno055.sensor_data.eul_heading        = 0;
			bno055.sensor_data.eul_roll           = 0;
			bno055.sensor_data.eul_pitch          = 0;
			bno055.sensor_data.qua_w              = 0;
			bno055.sensor_data.qua_x              = 0;
			bno055.sensor_data.qua_y              = 0;
			bno055.sensor_data.qua_z              = 0;
			bno055.sensor_data.lia_x              = 0;
			bno055.sensor_data.lia_y              = 0;
			bno055.sensor_data.lia_z              = 0;
			bno055.sensor_data.grv_x              = 0;
			bno055.sensor_data.grv_y              = 0;
			bno055.sensor_data.grv_z              = 0;
			bno055.sensor_data.calibration_status = 0;
			
			opr_mode = 0b0111;
			break;
		}

		case IMU_V3_SENSOR_FUSION_ON_WITHOUT_MAGNETOMETER: {
			bno055.sensor_data.mag_x              = 0;
			bno055.sensor_data.mag_y              = 0;
			bno055.sensor_data.mag_z              = 0;

			opr_mode = 0b1000;
			break;
		}

		case IMU_V3_SENSOR_FUSION_ON_WITHOUT_FAST_MAGNETOMETER_CALIBRATION: {
			opr_mode = 0b1011;
			break;
		}

		default:
		case IMU_V3_SENSOR_FUSION_ON: {
			opr_mode = 0b1100;
			break;
		}
	}

	bno055_task_write_register_with_check(BNO055_REG_OPR_MODE, &opr_mode, 1, 8);
}

void bno055_task_update_clock_config(void) {
	// First we go to config mode
	uint8_t opr_mode = 0b000;
	bno055_task_write_register_with_check(BNO055_REG_OPR_MODE, &opr_mode, 1, 20);

	uint8_t sys_trigger = 1 << 7;
	bno055_task_write_register_with_check(BNO055_REG_SYS_TRIGGER, &sys_trigger, 1, 0);

	bno055_task_update_sensor_fusion_mode(true);
}

void bno055_task_update_sensor_configuration(void) {
	// First we go to config mode
	uint8_t opr_mode = 0b000;
	bno055_task_write_register_with_check(BNO055_REG_OPR_MODE, &opr_mode, 1, 20);

	// Change page id to 1 to change sensor configurations
	uint8_t page_id = 1;
	bno055_task_write_register_with_check(BNO055_REG_PAGE_ID, &page_id, 1, 20);

	const uint8_t config[4] = {
		((bno055.accelerometer_range & 0b11) << 0) | ((bno055.accelerometer_bandwidth & 0b111) << 2) | (0b000 << 5), // ACC
		((bno055.magnetometer_rate & 0b111) << 0) | (0b01 << 3) | (0b11 << 5),                                       // MAG
		((bno055.gyroscope_range & 0b111) << 0) | ((bno055.gyroscope_bandwidth & 0b111) << 3),                       // GYR 1
		(0b000 << 0)                                                                                                 // GYR 2
	};

	bno055_task_write_register_with_check(BNO055_REG_ACC_CONFIG, config, 4, 0);

	// Change page id back
	page_id = 0;
	bno055_task_write_register_with_check(BNO055_REG_PAGE_ID, &page_id, 1, 0);

	bno055_task_update_sensor_fusion_mode(true);
}

void bno055_task_load_calibration(void) {
	uint32_t page[EEPROM_PAGE_SIZE/sizeof(uint32_t)];
	bootloader_read_eeprom_page(BNO055_CALIBRATION_PAGE, page);

	BNO055Calibration *calibration = (BNO055Calibration*)page;

	if(calibration->magic == BNO055_CALIBRATION_MAGIC) {
		logd("Read calibration from flash and save to BNO055:\n\r");
		logd(" Acc Offset: %d %d %d\n\r", calibration->acc_offset[0], calibration->acc_offset[1], calibration->acc_offset[2]);
		logd(" Mag Offset: %d %d %d\n\r", calibration->mag_offset[0], calibration->mag_offset[1], calibration->mag_offset[2]);
		logd(" Gyr Offset: %d %d %d\n\r", calibration->gyr_offset[0], calibration->gyr_offset[1], calibration->gyr_offset[2]);
		logd(" Acc Radius: %d\n\r", calibration->acc_radius);
		logd(" Mag Radius: %d\n\r", calibration->mag_radius);
		bno055_task_write_register_with_check(BNO055_REG_ACC_OFFSET_X_LSB, (uint8_t *)calibration->acc_offset, 3*2, 20);
		bno055_task_write_register_with_check(BNO055_REG_MAG_OFFSET_X_LSB, (uint8_t *)calibration->mag_offset, 3*2, 20);
		bno055_task_write_register_with_check(BNO055_REG_GYR_OFFSET_X_LSB, (uint8_t *)calibration->gyr_offset, 3*2, 20);
		bno055_task_write_register_with_check(BNO055_REG_ACC_RADIUS_LSB, (uint8_t *)&calibration->acc_radius, 2*2, 20);
	} else {
		logd("No calibration found\n\r");
	}

	bno055_task_update_sensor_fusion_mode(true);
}

void bno055_task_save_calibration(void) {
	if(bno055.sensor_data.calibration_status != 0xFF) {
		return;
	}

	uint8_t opr_mode = 0b00000000;
	bno055_task_write_register_with_check(BNO055_REG_OPR_MODE, &opr_mode, 1, 20);
	BNO055Calibration calibration = {{0}};
	bno055_task_read_register(BNO055_REG_ACC_OFFSET_X_LSB, (uint8_t *)calibration.acc_offset, 3*2);
	bno055_task_read_register(BNO055_REG_MAG_OFFSET_X_LSB, (uint8_t *)calibration.mag_offset, 3*2);
	bno055_task_read_register(BNO055_REG_GYR_OFFSET_X_LSB, (uint8_t *)calibration.gyr_offset, 3*2);
	bno055_task_read_register(BNO055_REG_ACC_RADIUS_LSB, (uint8_t *)&calibration.acc_radius, 2*2);
	calibration.magic = BNO055_CALIBRATION_MAGIC;
	bno055_task_update_sensor_fusion_mode(true);

	logd("Read calibration from BNO055 and save to flash:\n\r");
	logd(" Acc Offset: %d %d %d\n\r", calibration.acc_offset[0], calibration.acc_offset[1], calibration.acc_offset[2]);
	logd(" Mag Offset: %d %d %d\n\r", calibration.mag_offset[0], calibration.mag_offset[1], calibration.mag_offset[2]);
	logd(" Gyr Offset: %d %d %d\n\r", calibration.gyr_offset[0], calibration.gyr_offset[1], calibration.gyr_offset[2]);
	logd(" Acc Radius: %d\n\r", calibration.acc_radius);
	logd(" Mag Radius: %d\n\r", calibration.mag_radius);

	uint32_t page[EEPROM_PAGE_SIZE/sizeof(uint32_t)] = {0};
	memcpy(page, &calibration, sizeof(BNO055Calibration));
	bootloader_write_eeprom_page(BNO055_CALIBRATION_PAGE, page);
}

void bno055_task_reset(void) {
	XMC_GPIO_SetOutputLow(BNO055_NRESET);
	coop_task_sleep_ms(500);
	XMC_GPIO_SetOutputHigh(BNO055_NRESET);
	// From off to config mode   -> 400ms
	// From reset to config mode -> 650ms
	coop_task_sleep_ms(700);
}

void bno055_tick_task(void) {
	bno055_task_reset();

#if LOGGING_LEVEL == LOGGING_DEBUG
	uint8_t chip_id;
	bno055_task_read_register(BNO055_REG_CHIP_ID, &chip_id, 1);
	uint8_t version[2];
	bno055_task_read_register(BNO055_REG_SW_REV_ID_MSB, version, 2);
	logd("BNO055 Chip ID: %d, Version: %d.%d\n\r", chip_id, version[0], version[1]);
#endif

	bno055_task_load_calibration();

	while(true) {
		if(bno055.new_clock_config) {
			bno055_task_update_clock_config();
			bno055.new_clock_config = false;
		} else if(bno055.new_configuration) {
			bno055_task_update_sensor_configuration();
			bno055.new_configuration = false;
		} else if(bno055.new_sensor_fusion_mode) {
			bno055_task_update_sensor_fusion_mode(false);
			bno055.new_sensor_fusion_mode = false;
		} else if(bno055.new_calibration) {
			bno055_task_save_calibration();
			bno055.new_calibration = false;
		} else {
			bno055_task_update_sensor_data();
		}
		coop_task_yield();
	}
}

void bno055_tick(void) {
	coop_task_tick(&bno055_task);
}

void bno055_init(void) {
	memset(&bno055, 0, sizeof(BNO055));

	bno055.magnetometer_rate       = IMU_V3_MAGNETOMETER_RATE_20HZ;
	bno055.gyroscope_range         = IMU_V3_GYROSCOPE_RANGE_2000DPS;
	bno055.gyroscope_bandwidth     = IMU_V3_GYROSCOPE_BANDWIDTH_32HZ;
	bno055.accelerometer_range     = IMU_V3_ACCELEROMETER_RANGE_4G;
	bno055.accelerometer_bandwidth = IMU_V3_ACCELEROMETER_BANDWIDTH_62_5HZ;
	bno055.sensor_fusion_mode      = IMU_V3_SENSOR_FUSION_ON;

	bno055.new_configuration       = true;
	bno055.new_clock_config        = true;
	bno055.new_sensor_fusion_mode  = false;

	const XMC_GPIO_CONFIG_t config_output_high = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	XMC_GPIO_Init(BNO055_ADDRESS_SELECTION, &config_output_high);
	XMC_GPIO_Init(BNO055_NBOOT, &config_output_high);
	XMC_GPIO_Init(BNO055_NRESET, &config_output_high);

	bno055_init_i2c();
	coop_task_init(&bno055_task, bno055_tick_task);
}