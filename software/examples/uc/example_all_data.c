// This example is not self-contained.
// It requres usage of the example driver specific to your platform.
// See the HAL documentation.

#include "bindings/hal_common.h"
#include "bindings/bricklet_imu_v3.h"

#define UID "XYZ" // Change XYZ to the UID of your IMU Bricklet 3.0

void check(int rc, const char* msg);

void example_setup(TF_HalContext *hal);
void example_loop(TF_HalContext *hal);


// Callback function for all data callback
static void all_data_handler(TF_IMUV3 *device, int16_t acceleration[3],
                             int16_t magnetic_field[3], int16_t angular_velocity[3],
                             int16_t euler_angle[3], int16_t quaternion[4],
                             int16_t linear_acceleration[3], int16_t gravity_vector[3],
                             int8_t temperature, uint8_t calibration_status,
                             void *user_data) {
	(void)device; (void)user_data; // avoid unused parameter warning

	tf_hal_printf("Acceleration [X]: %d 1/%d m/s²\n", acceleration[0], 100);
	tf_hal_printf("Acceleration [Y]: %d 1/%d m/s²\n", acceleration[1], 100);
	tf_hal_printf("Acceleration [Z]: %d 1/%d m/s²\n", acceleration[2], 100);
	tf_hal_printf("Magnetic Field [X]: %d 1/%d µT\n", magnetic_field[0], 16);
	tf_hal_printf("Magnetic Field [Y]: %d 1/%d µT\n", magnetic_field[1], 16);
	tf_hal_printf("Magnetic Field [Z]: %d 1/%d µT\n", magnetic_field[2], 16);
	tf_hal_printf("Angular Velocity [X]: %d 1/%d °/s\n", angular_velocity[0], 16);
	tf_hal_printf("Angular Velocity [Y]: %d 1/%d °/s\n", angular_velocity[1], 16);
	tf_hal_printf("Angular Velocity [Z]: %d 1/%d °/s\n", angular_velocity[2], 16);
	tf_hal_printf("Euler Angle [Heading]: %d 1/%d °\n", euler_angle[0], 16);
	tf_hal_printf("Euler Angle [Roll]: %d 1/%d °\n", euler_angle[1], 16);
	tf_hal_printf("Euler Angle [Pitch]: %d 1/%d °\n", euler_angle[2], 16);
	tf_hal_printf("Quaternion [W]: %d 1/%d\n", quaternion[0], 16383);
	tf_hal_printf("Quaternion [X]: %d 1/%d\n", quaternion[1], 16383);
	tf_hal_printf("Quaternion [Y]: %d 1/%d\n", quaternion[2], 16383);
	tf_hal_printf("Quaternion [Z]: %d 1/%d\n", quaternion[3], 16383);
	tf_hal_printf("Linear Acceleration [X]: %d 1/%d m/s²\n", linear_acceleration[0], 100);
	tf_hal_printf("Linear Acceleration [Y]: %d 1/%d m/s²\n", linear_acceleration[1], 100);
	tf_hal_printf("Linear Acceleration [Z]: %d 1/%d m/s²\n", linear_acceleration[2], 100);
	tf_hal_printf("Gravity Vector [X]: %d 1/%d m/s²\n", gravity_vector[0], 100);
	tf_hal_printf("Gravity Vector [Y]: %d 1/%d m/s²\n", gravity_vector[1], 100);
	tf_hal_printf("Gravity Vector [Z]: %d 1/%d m/s²\n", gravity_vector[2], 100);
	tf_hal_printf("Temperature: %I8d °C\n", temperature);
	tf_hal_printf("Calibration Status: %I8u\n", calibration_status);
	tf_hal_printf("\n");
}

static TF_IMUV3 imu;

void example_setup(TF_HalContext *hal) {
	// Create device object
	check(tf_imu_v3_create(&imu, UID, hal), "create device object");

	// Register all data callback to function all_data_handler
	tf_imu_v3_register_all_data_callback(&imu,
	                                     all_data_handler,
	                                     NULL);

	// Set period for all data callback to 0.1s (100ms)
	tf_imu_v3_set_all_data_callback_configuration(&imu, 100, false);
}

void example_loop(TF_HalContext *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
