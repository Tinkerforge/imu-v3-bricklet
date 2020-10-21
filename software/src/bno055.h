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

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/hal/i2c_fifo/i2c_fifo.h"

#define BNO055_CALIBRATION_PAGE    1
#define BNO055_CALIBRATION_MAGIC   0xDEADBEEF
#define BNO055_CALIBRATION_LENGTH  (sizeof(BNO055Calibration) - sizeof(uint32_t))

#define BNO055_CALLBACK_NUM                  9
#define BNO055_CALLBACK_ACCELERATION         0
#define BNO055_CALLBACK_MAGNETIC_FIELD       1
#define BNO055_CALLBACK_ANGULAR_VELOCITY     2
#define BNO055_CALLBACK_TEMPERATURE          3
#define BNO055_CALLBACK_ORIENTATION          4
#define BNO055_CALLBACK_LINEAR_ACCELERATION  5
#define BNO055_CALLBACK_GRAVITY_VECTOR       6
#define BNO055_CALLBACK_QUATERNION           7
#define BNO055_CALLBACK_ALL_DATA             8

typedef struct {
	int16_t acc_offset[3];
	int16_t mag_offset[3];
	int16_t gyr_offset[3];
	int16_t acc_radius;
	int16_t mag_radius;
	uint32_t magic;
} __attribute__((packed)) BNO055Calibration;

typedef struct {
	int16_t acc_x; // 1m/s^2 = 100 LSB
	int16_t acc_y;
	int16_t acc_z;
	int16_t mag_x; // 1µT = 16 LSB
	int16_t mag_y;
	int16_t mag_z;
	int16_t gyr_x; // 1dps (degree-per-second) = 16 LSB
	int16_t gyr_y;
	int16_t gyr_z;
	int16_t eul_heading; // 1° = 16 LSB
	int16_t eul_roll;
	int16_t eul_pitch;
	uint16_t qua_w; // 1 Quaternion = 2^14 LSB
	uint16_t qua_x;
	uint16_t qua_y;
	uint16_t qua_z;
	int16_t lia_x; // 1m/s^2 = 100 LSB
	int16_t lia_y;
	int16_t lia_z;
	int16_t grv_x; // 1m/s^2 = 100 LSB
	int16_t grv_y;
	int16_t grv_z;
	int8_t temperature; // 1°C = 1 LSB
	uint8_t calibration_status;
} __attribute__((packed)) SensorData;

typedef struct {
	bool new_configuration;
	bool new_sensor_fusion_mode;
	bool new_calibration;
	bool new_clock_config;


	uint8_t magnetometer_rate;
	uint8_t gyroscope_range;
	uint8_t gyroscope_bandwidth;
	uint8_t accelerometer_range;
	uint8_t accelerometer_bandwidth;
	uint8_t sensor_fusion_mode;

	uint8_t update_sensor_counter;
	uint32_t update_sensor_start;

	bool sensor_done_copied;
	SensorData sensor_data;
	SensorData sensor_data_ready;

	I2CFifo i2c_fifo;

	uint32_t cb_period[BNO055_CALLBACK_NUM];
	bool cb_vhtc[BNO055_CALLBACK_NUM];
} BNO055;

extern BNO055 bno055;

void bno055_tick(void);
void bno055_init(void);


#define BNO055_REG_PAGE_ID          0x07

// Page 0
#define BNO055_REG_CHIP_ID          0x00
#define BNO055_REG_ACC_ID           0x01
#define BNO055_REG_MAG_ID           0x02
#define BNO055_REG_GYR_ID           0x03
#define BNO055_REG_SW_REV_ID_LSB    0x04
#define BNO055_REG_SW_REV_ID_MSB    0x05
#define BNO055_REG_BL_REV_ID        0x06
#define BNO055_REG_ACC_DATA_X_LSB   0x08
#define BNO055_REG_ACC_DATA_X_MSB   0x09
#define BNO055_REG_ACC_DATA_Y_LSB   0x0A
#define BNO055_REG_ACC_DATA_Y_MSB   0x0B
#define BNO055_REG_ACC_DATA_Z_LSB   0x0C
#define BNO055_REG_ACC_DATA_Z_MSB   0x0D
#define BNO055_REG_MAG_DATA_X_LSB   0x0E
#define BNO055_REG_MAG_DATA_X_MSB   0x0F
#define BNO055_REG_MAG_DATA_Y_LSB   0x10
#define BNO055_REG_MAG_DATA_Y_MSB   0x11
#define BNO055_REG_MAG_DATA_Z_LSB   0x12
#define BNO055_REG_MAG_DATA_Z_MSB   0x13
#define BNO055_REG_GYR_DATA_X_LSB   0x14
#define BNO055_REG_GYR_DATA_X_MSB   0x15
#define BNO055_REG_GYR_DATA_Y_LSB   0x16
#define BNO055_REG_GYR_DATA_Y_MSB   0x17
#define BNO055_REG_GYR_DATA_Z_LSB   0x18
#define BNO055_REG_GYR_DATA_Z_MSB   0x19
#define BNO055_REG_EUL_HEADING_LSB  0x1A
#define BNO055_REG_EUL_HEADING_MSB  0x1B
#define BNO055_REG_EUL_ROLL_LSB     0x1C
#define BNO055_REG_EUL_ROLL_MSB     0x1D
#define BNO055_REG_EUL_PITCH_LSB    0x1E
#define BNO055_REG_EUL_PITCH_MSB    0x1F
#define BNO055_REG_QUA_DATA_W_LSB   0x20
#define BNO055_REG_QUA_DATA_W_MSB   0x21
#define BNO055_REG_QUA_DATA_X_LSB   0x22
#define BNO055_REG_QUA_DATA_X_MSB   0x23
#define BNO055_REG_QUA_DATA_Y_LSB   0x24
#define BNO055_REG_QUA_DATA_Y_MSB   0x25
#define BNO055_REG_QUA_DATA_Z_LSB   0x26
#define BNO055_REG_QUA_DATA_Z_MSB   0x27
#define BNO055_REG_LIA_DATA_X_LSB   0x28
#define BNO055_REG_LIA_DATA_X_MSB   0x29
#define BNO055_REG_LIA_DATA_Y_LSB   0x2A
#define BNO055_REG_LIA_DATA_Y_MSB   0x2B
#define BNO055_REG_LIA_DATA_Z_LSB   0x2C
#define BNO055_REG_LIA_DATA_Z_MSB   0x2D
#define BNO055_REG_GRV_DATA_X_LSB   0x2E
#define BNO055_REG_GRV_DATA_X_MSB   0x2F
#define BNO055_REG_GRV_DATA_Y_LSB   0x30
#define BNO055_REG_GRV_DATA_Y_MSB   0x31
#define BNO055_REG_GRV_DATA_Z_LSB   0x32
#define BNO055_REG_GRV_DATA_Z_MSB   0x33
#define BNO055_REG_TEMP             0x34
#define BNO055_REG_CALIB_STAT       0x35
#define BNO055_REG_ST_RESULT        0x36
#define BNO055_REG_INT_STA          0x37
#define BNO055_REG_SYS_CLK_STATUS   0x38
#define BNO055_REG_SYS_STATUS       0x39
#define BNO055_REG_SYS_ERR          0x3A
#define BNO055_REG_UNIT_SEL         0x3B
#define BNO055_REG_OPR_MODE         0x3D
#define BNO055_REG_PWR_MODE         0x3E
#define BNO055_REG_SYS_TRIGGER      0x3F
#define BNO055_REG_TEMP_SOURCE      0x40
#define BNO055_REG_AXIS_MAP_CONFIG  0x41
#define BNO055_REG_AXIS_MAP_SIGN    0x42
#define BNO055_REG_ACC_OFFSET_X_LSB 0x55
#define BNO055_REG_ACC_OFFSET_X_MSB 0x56
#define BNO055_REG_ACC_OFFSET_Y_LSB 0x57
#define BNO055_REG_ACC_OFFSET_Y_MSB 0x58
#define BNO055_REG_ACC_OFFSET_Z_LSB 0x59
#define BNO055_REG_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_REG_MAG_OFFSET_X_LSB 0x5B
#define BNO055_REG_MAG_OFFSET_X_MSB 0x5C
#define BNO055_REG_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_REG_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_REG_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_REG_MAG_OFFSET_Z_MSB 0x60
#define BNO055_REG_GYR_OFFSET_X_LSB 0x61
#define BNO055_REG_GYR_OFFSET_X_MSB 0x62
#define BNO055_REG_GYR_OFFSET_Y_LSB 0x63
#define BNO055_REG_GYR_OFFSET_Y_MSB 0x64
#define BNO055_REG_GYR_OFFSET_Z_LSB 0x65
#define BNO055_REG_GYR_OFFSET_Z_MSB 0x66
#define BNO055_REG_ACC_RADIUS_LSB   0x67
#define BNO055_REG_ACC_RADIUS_MSB   0x68
#define BNO055_REG_MAG_RADIUS_LSB   0x69
#define BNO055_REG_MAG_RADIUS_MSB   0x6A

// Page 1
#define BNO055_REG_ACC_CONFIG       0x08
#define BNO055_REG_MAG_CONFIG       0x09
#define BNO055_REG_GYR_CONFIG_0     0x0A
#define BNO055_REG_GYR_CONFIG_1     0x0B
#define BNO055_REG_ACC_SLEEP_CONFIG 0x0C
#define BNO055_REG_GYR_SLEEP_CONFIG 0x0D
#define BNO055_REG_INT_MSK          0x0F
#define BNO055_REG_INT_EN           0x10
#define BNO055_REG_ACC_AM_THRES     0x11
#define BNO055_REG_ACC_INT_SETTINGS 0x12
#define BNO055_REG_ACC_HG_DURATION  0x13
#define BNO055_REG_ACC_HG_THRES     0x14
#define BNO055_REG_ACC_NM_THRES     0x15
#define BNO055_REG_ACC_NM_SET       0x16
#define BNO055_REG_GYR_INT_SETTINGS 0x17
#define BNO055_REG_GYR_HR_X_SET     0x18
#define BNO055_REG_GYR_DUR_X        0x19
#define BNO055_REG_GYR_HR_Y_SET     0x1A
#define BNO055_REG_GYR_DUR_Y        0x1B
#define BNO055_REG_GYR_HR_Z_SET     0x1C
#define BNO055_REG_GYR_DUR_Z        0x1D
#define BNO055_REG_GYR_AM_THRES     0x1E
#define BNO055_REG_GYR_AM_SET       0x1F
#define BNO055_REG_UNIQUE_ID_0      0x50
#define BNO055_REG_UNIQUE_ID_1      0x51
#define BNO055_REG_UNIQUE_ID_2      0x52
#define BNO055_REG_UNIQUE_ID_3      0x53
#define BNO055_REG_UNIQUE_ID_4      0x54
#define BNO055_REG_UNIQUE_ID_5      0x55
#define BNO055_REG_UNIQUE_ID_6      0x56
#define BNO055_REG_UNIQUE_ID_7      0x57
#define BNO055_REG_UNIQUE_ID_8      0x58
#define BNO055_REG_UNIQUE_ID_9      0x59
#define BNO055_REG_UNIQUE_ID_A      0x5A
#define BNO055_REG_UNIQUE_ID_B      0x5B
#define BNO055_REG_UNIQUE_ID_C      0x5C
#define BNO055_REG_UNIQUE_ID_D      0x5D
#define BNO055_REG_UNIQUE_ID_E      0x5E
#define BNO055_REG_UNIQUE_ID_F      0x5F

#endif