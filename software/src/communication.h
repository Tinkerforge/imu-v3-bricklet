/* imu-v3-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.h: TFP protocol message handling
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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/bootloader/bootloader.h"

// Default functions
BootloaderHandleMessageResponse handle_message(const void *data, void *response);
void communication_tick(void);
void communication_init(void);

// Constants

#define IMU_V3_MAGNETOMETER_RATE_2HZ 0
#define IMU_V3_MAGNETOMETER_RATE_6HZ 1
#define IMU_V3_MAGNETOMETER_RATE_8HZ 2
#define IMU_V3_MAGNETOMETER_RATE_10HZ 3
#define IMU_V3_MAGNETOMETER_RATE_15HZ 4
#define IMU_V3_MAGNETOMETER_RATE_20HZ 5
#define IMU_V3_MAGNETOMETER_RATE_25HZ 6
#define IMU_V3_MAGNETOMETER_RATE_30HZ 7

#define IMU_V3_GYROSCOPE_RANGE_2000DPS 0
#define IMU_V3_GYROSCOPE_RANGE_1000DPS 1
#define IMU_V3_GYROSCOPE_RANGE_500DPS 2
#define IMU_V3_GYROSCOPE_RANGE_250DPS 3
#define IMU_V3_GYROSCOPE_RANGE_125DPS 4

#define IMU_V3_GYROSCOPE_BANDWIDTH_523HZ 0
#define IMU_V3_GYROSCOPE_BANDWIDTH_230HZ 1
#define IMU_V3_GYROSCOPE_BANDWIDTH_116HZ 2
#define IMU_V3_GYROSCOPE_BANDWIDTH_47HZ 3
#define IMU_V3_GYROSCOPE_BANDWIDTH_23HZ 4
#define IMU_V3_GYROSCOPE_BANDWIDTH_12HZ 5
#define IMU_V3_GYROSCOPE_BANDWIDTH_64HZ 6
#define IMU_V3_GYROSCOPE_BANDWIDTH_32HZ 7

#define IMU_V3_ACCELEROMETER_RANGE_2G 0
#define IMU_V3_ACCELEROMETER_RANGE_4G 1
#define IMU_V3_ACCELEROMETER_RANGE_8G 2
#define IMU_V3_ACCELEROMETER_RANGE_16G 3

#define IMU_V3_ACCELEROMETER_BANDWIDTH_7_81HZ 0
#define IMU_V3_ACCELEROMETER_BANDWIDTH_15_63HZ 1
#define IMU_V3_ACCELEROMETER_BANDWIDTH_31_25HZ 2
#define IMU_V3_ACCELEROMETER_BANDWIDTH_62_5HZ 3
#define IMU_V3_ACCELEROMETER_BANDWIDTH_125HZ 4
#define IMU_V3_ACCELEROMETER_BANDWIDTH_250HZ 5
#define IMU_V3_ACCELEROMETER_BANDWIDTH_500HZ 6
#define IMU_V3_ACCELEROMETER_BANDWIDTH_1000HZ 7

#define IMU_V3_SENSOR_FUSION_OFF 0
#define IMU_V3_SENSOR_FUSION_ON 1
#define IMU_V3_SENSOR_FUSION_ON_WITHOUT_MAGNETOMETER 2
#define IMU_V3_SENSOR_FUSION_ON_WITHOUT_FAST_MAGNETOMETER_CALIBRATION 3

#define IMU_V3_BOOTLOADER_MODE_BOOTLOADER 0
#define IMU_V3_BOOTLOADER_MODE_FIRMWARE 1
#define IMU_V3_BOOTLOADER_MODE_BOOTLOADER_WAIT_FOR_REBOOT 2
#define IMU_V3_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_REBOOT 3
#define IMU_V3_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT 4

#define IMU_V3_BOOTLOADER_STATUS_OK 0
#define IMU_V3_BOOTLOADER_STATUS_INVALID_MODE 1
#define IMU_V3_BOOTLOADER_STATUS_NO_CHANGE 2
#define IMU_V3_BOOTLOADER_STATUS_ENTRY_FUNCTION_NOT_PRESENT 3
#define IMU_V3_BOOTLOADER_STATUS_DEVICE_IDENTIFIER_INCORRECT 4
#define IMU_V3_BOOTLOADER_STATUS_CRC_MISMATCH 5

#define IMU_V3_STATUS_LED_CONFIG_OFF 0
#define IMU_V3_STATUS_LED_CONFIG_ON 1
#define IMU_V3_STATUS_LED_CONFIG_SHOW_HEARTBEAT 2
#define IMU_V3_STATUS_LED_CONFIG_SHOW_STATUS 3

// Function and callback IDs and structs
#define FID_GET_ACCELERATION 1
#define FID_GET_MAGNETIC_FIELD 2
#define FID_GET_ANGULAR_VELOCITY 3
#define FID_GET_TEMPERATURE 4
#define FID_GET_ORIENTATION 5
#define FID_GET_LINEAR_ACCELERATION 6
#define FID_GET_GRAVITY_VECTOR 7
#define FID_GET_QUATERNION 8
#define FID_GET_ALL_DATA 9
#define FID_SAVE_CALIBRATION 10
#define FID_SET_SENSOR_CONFIGURATION 11
#define FID_GET_SENSOR_CONFIGURATION 12
#define FID_SET_SENSOR_FUSION_MODE 13
#define FID_GET_SENSOR_FUSION_MODE 14
#define FID_SET_ACCELERATION_CALLBACK_CONFIGURATION 15
#define FID_GET_ACCELERATION_CALLBACK_CONFIGURATION 16
#define FID_SET_MAGNETIC_FIELD_CALLBACK_CONFIGURATION 17
#define FID_GET_MAGNETIC_FIELD_CALLBACK_CONFIGURATION 18
#define FID_SET_ANGULAR_VELOCITY_CALLBACK_CONFIGURATION 19
#define FID_GET_ANGULAR_VELOCITY_CALLBACK_CONFIGURATION 20
#define FID_SET_TEMPERATURE_CALLBACK_CONFIGURATION 21
#define FID_GET_TEMPERATURE_CALLBACK_CONFIGURATION 22
#define FID_SET_ORIENTATION_CALLBACK_CONFIGURATION 23
#define FID_GET_ORIENTATION_CALLBACK_CONFIGURATION 24
#define FID_SET_LINEAR_ACCELERATION_CALLBACK_CONFIGURATION 25
#define FID_GET_LINEAR_ACCELERATION_CALLBACK_CONFIGURATION 26
#define FID_SET_GRAVITY_VECTOR_CALLBACK_CONFIGURATION 27
#define FID_GET_GRAVITY_VECTOR_CALLBACK_CONFIGURATION 28
#define FID_SET_QUATERNION_CALLBACK_CONFIGURATION 29
#define FID_GET_QUATERNION_CALLBACK_CONFIGURATION 30
#define FID_SET_ALL_DATA_CALLBACK_CONFIGURATION 31
#define FID_GET_ALL_DATA_CALLBACK_CONFIGURATION 32

#define FID_CALLBACK_ACCELERATION 33
#define FID_CALLBACK_MAGNETIC_FIELD 34
#define FID_CALLBACK_ANGULAR_VELOCITY 35
#define FID_CALLBACK_TEMPERATURE 36
#define FID_CALLBACK_LINEAR_ACCELERATION 37
#define FID_CALLBACK_GRAVITY_VECTOR 38
#define FID_CALLBACK_ORIENTATION 39
#define FID_CALLBACK_QUATERNION 40
#define FID_CALLBACK_ALL_DATA 41

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAcceleration;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetAcceleration_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMagneticField;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetMagneticField_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAngularVelocity;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetAngularVelocity_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetTemperature;

typedef struct {
	TFPMessageHeader header;
	int8_t temperature;
} __attribute__((__packed__)) GetTemperature_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetOrientation;

typedef struct {
	TFPMessageHeader header;
	int16_t heading;
	int16_t roll;
	int16_t pitch;
} __attribute__((__packed__)) GetOrientation_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetLinearAcceleration;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetLinearAcceleration_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetGravityVector;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetGravityVector_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetQuaternion;

typedef struct {
	TFPMessageHeader header;
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GetQuaternion_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAllData;

typedef struct {
	TFPMessageHeader header;
	int16_t acceleration[3];
	int16_t magnetic_field[3];
	int16_t angular_velocity[3];
	int16_t euler_angle[3];
	int16_t quaternion[4];
	int16_t linear_acceleration[3];
	int16_t gravity_vector[3];
	int8_t temperature;
	uint8_t calibration_status;
} __attribute__((__packed__)) GetAllData_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) SaveCalibration;

typedef struct {
	TFPMessageHeader header;
	bool calibration_done;
} __attribute__((__packed__)) SaveCalibration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t magnetometer_rate;
	uint8_t gyroscope_range;
	uint8_t gyroscope_bandwidth;
	uint8_t accelerometer_range;
	uint8_t accelerometer_bandwidth;
} __attribute__((__packed__)) SetSensorConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSensorConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t magnetometer_rate;
	uint8_t gyroscope_range;
	uint8_t gyroscope_bandwidth;
	uint8_t accelerometer_range;
	uint8_t accelerometer_bandwidth;
} __attribute__((__packed__)) GetSensorConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t mode;
} __attribute__((__packed__)) SetSensorFusionMode;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSensorFusionMode;

typedef struct {
	TFPMessageHeader header;
	uint8_t mode;
} __attribute__((__packed__)) GetSensorFusionMode_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetAccelerationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAccelerationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetAccelerationCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetMagneticFieldCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMagneticFieldCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetMagneticFieldCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetAngularVelocityCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAngularVelocityCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetAngularVelocityCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetTemperatureCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetTemperatureCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetTemperatureCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetOrientationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetOrientationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetOrientationCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetLinearAccelerationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetLinearAccelerationCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetLinearAccelerationCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetGravityVectorCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetGravityVectorCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetGravityVectorCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetQuaternionCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetQuaternionCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetQuaternionCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) SetAllDataCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAllDataCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
	bool value_has_to_change;
} __attribute__((__packed__)) GetAllDataCallbackConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) Acceleration_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) MagneticField_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) AngularVelocity_Callback;

typedef struct {
	TFPMessageHeader header;
	int8_t temperature;
} __attribute__((__packed__)) Temperature_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) LinearAcceleration_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) GravityVector_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t heading;
	int16_t roll;
	int16_t pitch;
} __attribute__((__packed__)) Orientation_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((__packed__)) Quaternion_Callback;

typedef struct {
	TFPMessageHeader header;
	int16_t acceleration[3];
	int16_t magnetic_field[3];
	int16_t angular_velocity[3];
	int16_t euler_angle[3];
	int16_t quaternion[4];
	int16_t linear_acceleration[3];
	int16_t gravity_vector[3];
	int8_t temperature;
	uint8_t calibration_status;
} __attribute__((__packed__)) AllData_Callback;


// Function prototypes
BootloaderHandleMessageResponse get_acceleration(const GetAcceleration *data, GetAcceleration_Response *response);
BootloaderHandleMessageResponse get_magnetic_field(const GetMagneticField *data, GetMagneticField_Response *response);
BootloaderHandleMessageResponse get_angular_velocity(const GetAngularVelocity *data, GetAngularVelocity_Response *response);
BootloaderHandleMessageResponse get_temperature(const GetTemperature *data, GetTemperature_Response *response);
BootloaderHandleMessageResponse get_orientation(const GetOrientation *data, GetOrientation_Response *response);
BootloaderHandleMessageResponse get_linear_acceleration(const GetLinearAcceleration *data, GetLinearAcceleration_Response *response);
BootloaderHandleMessageResponse get_gravity_vector(const GetGravityVector *data, GetGravityVector_Response *response);
BootloaderHandleMessageResponse get_quaternion(const GetQuaternion *data, GetQuaternion_Response *response);
BootloaderHandleMessageResponse get_all_data(const GetAllData *data, GetAllData_Response *response);
BootloaderHandleMessageResponse save_calibration(const SaveCalibration *data, SaveCalibration_Response *response);
BootloaderHandleMessageResponse set_sensor_configuration(const SetSensorConfiguration *data);
BootloaderHandleMessageResponse get_sensor_configuration(const GetSensorConfiguration *data, GetSensorConfiguration_Response *response);
BootloaderHandleMessageResponse set_sensor_fusion_mode(const SetSensorFusionMode *data);
BootloaderHandleMessageResponse get_sensor_fusion_mode(const GetSensorFusionMode *data, GetSensorFusionMode_Response *response);
BootloaderHandleMessageResponse set_acceleration_callback_configuration(const SetAccelerationCallbackConfiguration *data);
BootloaderHandleMessageResponse get_acceleration_callback_configuration(const GetAccelerationCallbackConfiguration *data, GetAccelerationCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_magnetic_field_callback_configuration(const SetMagneticFieldCallbackConfiguration *data);
BootloaderHandleMessageResponse get_magnetic_field_callback_configuration(const GetMagneticFieldCallbackConfiguration *data, GetMagneticFieldCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_angular_velocity_callback_configuration(const SetAngularVelocityCallbackConfiguration *data);
BootloaderHandleMessageResponse get_angular_velocity_callback_configuration(const GetAngularVelocityCallbackConfiguration *data, GetAngularVelocityCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_temperature_callback_configuration(const SetTemperatureCallbackConfiguration *data);
BootloaderHandleMessageResponse get_temperature_callback_configuration(const GetTemperatureCallbackConfiguration *data, GetTemperatureCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_orientation_callback_configuration(const SetOrientationCallbackConfiguration *data);
BootloaderHandleMessageResponse get_orientation_callback_configuration(const GetOrientationCallbackConfiguration *data, GetOrientationCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_linear_acceleration_callback_configuration(const SetLinearAccelerationCallbackConfiguration *data);
BootloaderHandleMessageResponse get_linear_acceleration_callback_configuration(const GetLinearAccelerationCallbackConfiguration *data, GetLinearAccelerationCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_gravity_vector_callback_configuration(const SetGravityVectorCallbackConfiguration *data);
BootloaderHandleMessageResponse get_gravity_vector_callback_configuration(const GetGravityVectorCallbackConfiguration *data, GetGravityVectorCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_quaternion_callback_configuration(const SetQuaternionCallbackConfiguration *data);
BootloaderHandleMessageResponse get_quaternion_callback_configuration(const GetQuaternionCallbackConfiguration *data, GetQuaternionCallbackConfiguration_Response *response);
BootloaderHandleMessageResponse set_all_data_callback_configuration(const SetAllDataCallbackConfiguration *data);
BootloaderHandleMessageResponse get_all_data_callback_configuration(const GetAllDataCallbackConfiguration *data, GetAllDataCallbackConfiguration_Response *response);

// Callbacks
bool handle_acceleration_callback(void);
bool handle_magnetic_field_callback(void);
bool handle_angular_velocity_callback(void);
bool handle_temperature_callback(void);
bool handle_linear_acceleration_callback(void);
bool handle_gravity_vector_callback(void);
bool handle_orientation_callback(void);
bool handle_quaternion_callback(void);
bool handle_all_data_callback(void);

#define COMMUNICATION_CALLBACK_TICK_WAIT_MS 1
#define COMMUNICATION_CALLBACK_HANDLER_NUM 9
#define COMMUNICATION_CALLBACK_LIST_INIT \
	handle_acceleration_callback, \
	handle_magnetic_field_callback, \
	handle_angular_velocity_callback, \
	handle_temperature_callback, \
	handle_linear_acceleration_callback, \
	handle_gravity_vector_callback, \
	handle_orientation_callback, \
	handle_quaternion_callback, \
	handle_all_data_callback, \


#endif
