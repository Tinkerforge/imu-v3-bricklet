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
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}


BootloaderHandleMessageResponse get_acceleration(const GetAcceleration *data, GetAcceleration_Response *response) {
	response->header.length = sizeof(GetAcceleration_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_magnetic_field(const GetMagneticField *data, GetMagneticField_Response *response) {
	response->header.length = sizeof(GetMagneticField_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_angular_velocity(const GetAngularVelocity *data, GetAngularVelocity_Response *response) {
	response->header.length = sizeof(GetAngularVelocity_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_temperature(const GetTemperature *data, GetTemperature_Response *response) {
	response->header.length = sizeof(GetTemperature_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_orientation(const GetOrientation *data, GetOrientation_Response *response) {
	response->header.length = sizeof(GetOrientation_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_linear_acceleration(const GetLinearAcceleration *data, GetLinearAcceleration_Response *response) {
	response->header.length = sizeof(GetLinearAcceleration_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_gravity_vector(const GetGravityVector *data, GetGravityVector_Response *response) {
	response->header.length = sizeof(GetGravityVector_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_quaternion(const GetQuaternion *data, GetQuaternion_Response *response) {
	response->header.length = sizeof(GetQuaternion_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_all_data(const GetAllData *data, GetAllData_Response *response) {
	response->header.length = sizeof(GetAllData_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse save_calibration(const SaveCalibration *data, SaveCalibration_Response *response) {
	response->header.length = sizeof(SaveCalibration_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_sensor_configuration(const SetSensorConfiguration *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_sensor_configuration(const GetSensorConfiguration *data, GetSensorConfiguration_Response *response) {
	response->header.length = sizeof(GetSensorConfiguration_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_sensor_fusion_mode(const SetSensorFusionMode *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_sensor_fusion_mode(const GetSensorFusionMode *data, GetSensorFusionMode_Response *response) {
	response->header.length = sizeof(GetSensorFusionMode_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}





void communication_tick(void) {
//	communication_callback_tick();
}

void communication_init(void) {
//	communication_callback_init();
}
