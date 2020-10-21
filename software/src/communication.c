/* imu-v3-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.c: TFP protocol message handling
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

#include "communication.h"

#include "bricklib2/utility/communication_callback.h"
#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/hal/system_timer/system_timer.h"

#include "bno055.h"

BootloaderHandleMessageResponse handle_message(const void *message, void *response) {
	switch(tfp_get_fid_from_message(message)) {
		case FID_GET_ACCELERATION: return get_acceleration(message, response);
		case FID_GET_MAGNETIC_FIELD: return get_magnetic_field(message, response);
		case FID_GET_ANGULAR_VELOCITY: return get_angular_velocity(message, response);
		case FID_GET_TEMPERATURE: return get_temperature(message, response);
		case FID_GET_ORIENTATION: return get_orientation(message, response);
		case FID_GET_LINEAR_ACCELERATION: return get_linear_acceleration(message, response);
		case FID_GET_GRAVITY_VECTOR: return get_gravity_vector(message, response);
		case FID_GET_QUATERNION: return get_quaternion(message, response);
		case FID_GET_ALL_DATA: return get_all_data(message, response);
		case FID_SAVE_CALIBRATION: return save_calibration(message, response);
		case FID_SET_SENSOR_CONFIGURATION: return set_sensor_configuration(message);
		case FID_GET_SENSOR_CONFIGURATION: return get_sensor_configuration(message, response);
		case FID_SET_SENSOR_FUSION_MODE: return set_sensor_fusion_mode(message);
		case FID_GET_SENSOR_FUSION_MODE: return get_sensor_fusion_mode(message, response);
		case FID_SET_ACCELERATION_CALLBACK_CONFIGURATION: return set_acceleration_callback_configuration(message);
		case FID_GET_ACCELERATION_CALLBACK_CONFIGURATION: return get_acceleration_callback_configuration(message, response);
		case FID_SET_MAGNETIC_FIELD_CALLBACK_CONFIGURATION: return set_magnetic_field_callback_configuration(message);
		case FID_GET_MAGNETIC_FIELD_CALLBACK_CONFIGURATION: return get_magnetic_field_callback_configuration(message, response);
		case FID_SET_ANGULAR_VELOCITY_CALLBACK_CONFIGURATION: return set_angular_velocity_callback_configuration(message);
		case FID_GET_ANGULAR_VELOCITY_CALLBACK_CONFIGURATION: return get_angular_velocity_callback_configuration(message, response);
		case FID_SET_TEMPERATURE_CALLBACK_CONFIGURATION: return set_temperature_callback_configuration(message);
		case FID_GET_TEMPERATURE_CALLBACK_CONFIGURATION: return get_temperature_callback_configuration(message, response);
		case FID_SET_ORIENTATION_CALLBACK_CONFIGURATION: return set_orientation_callback_configuration(message);
		case FID_GET_ORIENTATION_CALLBACK_CONFIGURATION: return get_orientation_callback_configuration(message, response);
		case FID_SET_LINEAR_ACCELERATION_CALLBACK_CONFIGURATION: return set_linear_acceleration_callback_configuration(message);
		case FID_GET_LINEAR_ACCELERATION_CALLBACK_CONFIGURATION: return get_linear_acceleration_callback_configuration(message, response);
		case FID_SET_GRAVITY_VECTOR_CALLBACK_CONFIGURATION: return set_gravity_vector_callback_configuration(message);
		case FID_GET_GRAVITY_VECTOR_CALLBACK_CONFIGURATION: return get_gravity_vector_callback_configuration(message, response);
		case FID_SET_QUATERNION_CALLBACK_CONFIGURATION: return set_quaternion_callback_configuration(message);
		case FID_GET_QUATERNION_CALLBACK_CONFIGURATION: return get_quaternion_callback_configuration(message, response);
		case FID_SET_ALL_DATA_CALLBACK_CONFIGURATION: return set_all_data_callback_configuration(message);
		case FID_GET_ALL_DATA_CALLBACK_CONFIGURATION: return get_all_data_callback_configuration(message, response);
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}


BootloaderHandleMessageResponse get_acceleration(const GetAcceleration *data, GetAcceleration_Response *response) {
	response->header.length = sizeof(GetAcceleration_Response);
	response->x             = bno055.sensor_data.acc_x;
	response->y             = bno055.sensor_data.acc_y;
	response->z             = bno055.sensor_data.acc_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_magnetic_field(const GetMagneticField *data, GetMagneticField_Response *response) {
	response->header.length = sizeof(GetMagneticField_Response);
	response->x             = bno055.sensor_data.mag_x;
	response->y             = bno055.sensor_data.mag_y;
	response->z             = bno055.sensor_data.mag_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_angular_velocity(const GetAngularVelocity *data, GetAngularVelocity_Response *response) {
	response->header.length = sizeof(GetAngularVelocity_Response);
	response->x             = bno055.sensor_data.gyr_x;
	response->y             = bno055.sensor_data.gyr_y;
	response->z             = bno055.sensor_data.gyr_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_temperature(const GetTemperature *data, GetTemperature_Response *response) {
	response->header.length = sizeof(GetTemperature_Response);
	response->temperature   = bno055.sensor_data.temperature;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_orientation(const GetOrientation *data, GetOrientation_Response *response) {
	response->header.length = sizeof(GetOrientation_Response);
	response->heading       = bno055.sensor_data.eul_heading;
	response->pitch         = bno055.sensor_data.eul_pitch;
	response->roll          = bno055.sensor_data.eul_roll;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_linear_acceleration(const GetLinearAcceleration *data, GetLinearAcceleration_Response *response) {
	response->header.length = sizeof(GetLinearAcceleration_Response);
	response->x             = bno055.sensor_data.lia_x;
	response->y             = bno055.sensor_data.lia_y;
	response->z             = bno055.sensor_data.lia_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_gravity_vector(const GetGravityVector *data, GetGravityVector_Response *response) {
	response->header.length = sizeof(GetGravityVector_Response);
	response->x             = bno055.sensor_data.grv_x;
	response->y             = bno055.sensor_data.grv_y;
	response->z             = bno055.sensor_data.grv_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_quaternion(const GetQuaternion *data, GetQuaternion_Response *response) {
	response->header.length = sizeof(GetQuaternion_Response);
	response->w             = bno055.sensor_data.qua_w;
	response->x             = bno055.sensor_data.qua_x;
	response->y             = bno055.sensor_data.qua_y;
	response->z             = bno055.sensor_data.qua_z;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_all_data(const GetAllData *data, GetAllData_Response *response) {
	response->header.length = sizeof(GetAllData_Response);
	memcpy(response->acceleration, &bno055.sensor_data_ready, sizeof(SensorData));

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse save_calibration(const SaveCalibration *data, SaveCalibration_Response *response) {
	bno055.new_calibration     = bno055.sensor_data.calibration_status == 0xFF;

	response->header.length    = sizeof(SaveCalibration_Response);
	response->calibration_done = bno055.new_calibration;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_sensor_configuration(const SetSensorConfiguration *data) {
	if((data->magnetometer_rate       > IMU_V3_MAGNETOMETER_RATE_30HZ)   ||
	   (data->gyroscope_range         > IMU_V3_GYROSCOPE_RANGE_125DPS)   ||
	   (data->gyroscope_bandwidth     > IMU_V3_GYROSCOPE_BANDWIDTH_32HZ) ||
	   (data->accelerometer_range     > IMU_V3_ACCELEROMETER_RANGE_16G)  ||
	   (data->accelerometer_bandwidth > IMU_V3_ACCELEROMETER_BANDWIDTH_1000HZ)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	bno055.magnetometer_rate       = data->magnetometer_rate;
	bno055.gyroscope_range         = data->gyroscope_range;
	bno055.gyroscope_bandwidth     = data->gyroscope_bandwidth;
	bno055.accelerometer_range     = data->accelerometer_range;
	bno055.accelerometer_bandwidth = data->accelerometer_bandwidth;
	bno055.new_calibration         = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_sensor_configuration(const GetSensorConfiguration *data, GetSensorConfiguration_Response *response) {
	response->header.length           = sizeof(GetSensorConfiguration_Response);
	response->magnetometer_rate       = bno055.magnetometer_rate;
	response->gyroscope_range         = bno055.gyroscope_range;
	response->gyroscope_bandwidth     = bno055.gyroscope_bandwidth;
	response->accelerometer_range     = bno055.accelerometer_range;
	response->accelerometer_bandwidth = bno055.accelerometer_bandwidth;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_sensor_fusion_mode(const SetSensorFusionMode *data) {
	if(data->mode > IMU_V3_SENSOR_FUSION_ON_WITHOUT_FAST_MAGNETOMETER_CALIBRATION) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	bno055.sensor_fusion_mode     = data->mode;
	bno055.new_sensor_fusion_mode = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_sensor_fusion_mode(const GetSensorFusionMode *data, GetSensorFusionMode_Response *response) {
	response->header.length = sizeof(GetSensorFusionMode_Response);
	response->mode          = bno055.sensor_fusion_mode;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_acceleration_callback_configuration(const SetAccelerationCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_ACCELERATION] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_ACCELERATION]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_acceleration_callback_configuration(const GetAccelerationCallbackConfiguration *data, GetAccelerationCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetAccelerationCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_ACCELERATION];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_ACCELERATION];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_magnetic_field_callback_configuration(const SetMagneticFieldCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_MAGNETIC_FIELD] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_MAGNETIC_FIELD]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_magnetic_field_callback_configuration(const GetMagneticFieldCallbackConfiguration *data, GetMagneticFieldCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetMagneticFieldCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_MAGNETIC_FIELD];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_MAGNETIC_FIELD];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_angular_velocity_callback_configuration(const SetAngularVelocityCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_ANGULAR_VELOCITY] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_ANGULAR_VELOCITY]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_angular_velocity_callback_configuration(const GetAngularVelocityCallbackConfiguration *data, GetAngularVelocityCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetAngularVelocityCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_ANGULAR_VELOCITY];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_ANGULAR_VELOCITY];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_temperature_callback_configuration(const SetTemperatureCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_TEMPERATURE] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_TEMPERATURE]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_temperature_callback_configuration(const GetTemperatureCallbackConfiguration *data, GetTemperatureCallbackConfiguration_Response *response) {
	response->header.length = sizeof(GetTemperatureCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_TEMPERATURE];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_TEMPERATURE];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_orientation_callback_configuration(const SetOrientationCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_ORIENTATION] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_ORIENTATION]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_orientation_callback_configuration(const GetOrientationCallbackConfiguration *data, GetOrientationCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetOrientationCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_ORIENTATION];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_ORIENTATION];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_linear_acceleration_callback_configuration(const SetLinearAccelerationCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_LINEAR_ACCELERATION] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_LINEAR_ACCELERATION]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_linear_acceleration_callback_configuration(const GetLinearAccelerationCallbackConfiguration *data, GetLinearAccelerationCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetLinearAccelerationCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_LINEAR_ACCELERATION];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_LINEAR_ACCELERATION];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gravity_vector_callback_configuration(const SetGravityVectorCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_GRAVITY_VECTOR] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_GRAVITY_VECTOR]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gravity_vector_callback_configuration(const GetGravityVectorCallbackConfiguration *data, GetGravityVectorCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetGravityVectorCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_GRAVITY_VECTOR];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_GRAVITY_VECTOR];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_quaternion_callback_configuration(const SetQuaternionCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_QUATERNION] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_QUATERNION]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_quaternion_callback_configuration(const GetQuaternionCallbackConfiguration *data, GetQuaternionCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetQuaternionCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_QUATERNION];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_QUATERNION];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_all_data_callback_configuration(const SetAllDataCallbackConfiguration *data) {
	bno055.cb_period[BNO055_CALLBACK_ALL_DATA] = data->period;
	bno055.cb_vhtc[BNO055_CALLBACK_ALL_DATA]   = data->value_has_to_change;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_all_data_callback_configuration(const GetAllDataCallbackConfiguration *data, GetAllDataCallbackConfiguration_Response *response) {
	response->header.length       = sizeof(GetAllDataCallbackConfiguration_Response);
	response->period              = bno055.cb_period[BNO055_CALLBACK_ALL_DATA];
	response->value_has_to_change = bno055.cb_vhtc[BNO055_CALLBACK_ALL_DATA];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

bool check_imu_callback(uint8_t *data, uint8_t length, uint8_t callback_id) {
	static uint8_t  last_data_buffer[BNO055_CALLBACK_NUM][64] = {{0}};
	static uint32_t last_time[BNO055_CALLBACK_NUM] = {0};

	if((bno055.cb_period[callback_id] == 0) || !system_timer_is_time_elapsed_ms(last_time[callback_id], bno055.cb_period[callback_id])) {
		return false;
	}

	if(bno055.cb_vhtc[callback_id] && (memcmp(last_data_buffer[callback_id], data, length) == 0)) {
		return false;
	}

	memcpy(last_data_buffer[callback_id], data, length);
	last_time[callback_id] = system_timer_get_ms();

	return true;
}

bool handle_acceleration_callback(void) {
	static bool is_buffered = false;
	static Acceleration_Callback cb;

	if(!is_buffered) {
		cb.x = bno055.sensor_data.acc_x;
		cb.y = bno055.sensor_data.acc_y;
		cb.z = bno055.sensor_data.acc_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(Acceleration_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_ACCELERATION)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(Acceleration_Callback), FID_CALLBACK_ACCELERATION);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(Acceleration_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_magnetic_field_callback(void) {
	static bool is_buffered = false;
	static MagneticField_Callback cb;

	if(!is_buffered) {
		cb.x = bno055.sensor_data.mag_x;
		cb.y = bno055.sensor_data.mag_y;
		cb.z = bno055.sensor_data.mag_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(MagneticField_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_MAGNETIC_FIELD)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(MagneticField_Callback), FID_CALLBACK_MAGNETIC_FIELD);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(MagneticField_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_angular_velocity_callback(void) {
	static bool is_buffered = false;
	static AngularVelocity_Callback cb;

	if(!is_buffered) {
		cb.x = bno055.sensor_data.gyr_x;
		cb.y = bno055.sensor_data.gyr_y;
		cb.z = bno055.sensor_data.gyr_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(AngularVelocity_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_ANGULAR_VELOCITY)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(AngularVelocity_Callback), FID_CALLBACK_ANGULAR_VELOCITY);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(AngularVelocity_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_temperature_callback(void) {
	static bool is_buffered = false;
	static Temperature_Callback cb;

	if(!is_buffered) {
		cb.temperature = bno055.sensor_data.temperature;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(Temperature_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_TEMPERATURE)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(Temperature_Callback), FID_CALLBACK_TEMPERATURE);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(Temperature_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_linear_acceleration_callback(void) {
	static bool is_buffered = false;
	static LinearAcceleration_Callback cb;

	if(!is_buffered) {
		cb.x = bno055.sensor_data.lia_x;
		cb.y = bno055.sensor_data.lia_y;
		cb.z = bno055.sensor_data.lia_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(LinearAcceleration_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_LINEAR_ACCELERATION)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(LinearAcceleration_Callback), FID_CALLBACK_LINEAR_ACCELERATION);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(LinearAcceleration_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_gravity_vector_callback(void) {
	static bool is_buffered = false;
	static GravityVector_Callback cb;

	if(!is_buffered) {
		cb.x = bno055.sensor_data.grv_x;
		cb.y = bno055.sensor_data.grv_y;
		cb.z = bno055.sensor_data.grv_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(GravityVector_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_GRAVITY_VECTOR)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(GravityVector_Callback), FID_CALLBACK_GRAVITY_VECTOR);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(GravityVector_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_orientation_callback(void) {
	static bool is_buffered = false;
	static Orientation_Callback cb;

	if(!is_buffered) {
		cb.roll    = bno055.sensor_data.eul_roll;
		cb.pitch   = bno055.sensor_data.eul_pitch;
		cb.heading = bno055.sensor_data.eul_heading;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(Orientation_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_ORIENTATION)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(Orientation_Callback), FID_CALLBACK_ORIENTATION);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(Orientation_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_quaternion_callback(void) {
	static bool is_buffered = false;
	static Quaternion_Callback cb;

	if(!is_buffered) {
		cb.w = bno055.sensor_data.qua_w;
		cb.x = bno055.sensor_data.qua_x;
		cb.y = bno055.sensor_data.qua_y;
		cb.z = bno055.sensor_data.qua_z;
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(Quaternion_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_QUATERNION)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(Quaternion_Callback), FID_CALLBACK_QUATERNION);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(Quaternion_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_all_data_callback(void) {
	static bool is_buffered = false;
	static AllData_Callback cb;

	if(!is_buffered) {
		memcpy((uint8_t*)(&cb) + sizeof(TFPMessageHeader), &bno055.sensor_data, sizeof(SensorData));
		if(check_imu_callback((uint8_t*)(&cb) + sizeof(TFPMessageHeader), sizeof(AllData_Callback) - sizeof(TFPMessageHeader), BNO055_CALLBACK_ALL_DATA)) {
			tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(AllData_Callback), FID_CALLBACK_ALL_DATA);
		} else {
			return false;
		}
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(AllData_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

void communication_tick(void) {
	communication_callback_tick();
}

void communication_init(void) {
	communication_callback_init();
}
