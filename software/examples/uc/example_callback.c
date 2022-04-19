// This example is not self-contained.
// It requres usage of the example driver specific to your platform.
// See the HAL documentation.

#include "bindings/hal_common.h"
#include "bindings/bricklet_imu_v3.h"

#define UID "XYZ" // Change XYZ to the UID of your IMU Bricklet 3.0

void check(int rc, const char* msg);

void example_setup(TF_HAL *hal);
void example_loop(TF_HAL *hal);


// Callback function for quaternion callback
static void quaternion_handler(TF_IMUV3 *device, int16_t w, int16_t x, int16_t y,
                               int16_t z, void *user_data) {
	(void)device; (void)user_data; // avoid unused parameter warning

	tf_hal_printf("Quaternion [W]: %d 1/%d\n", w, 16383);
	tf_hal_printf("Quaternion [X]: %d 1/%d\n", x, 16383);
	tf_hal_printf("Quaternion [Y]: %d 1/%d\n", y, 16383);
	tf_hal_printf("Quaternion [Z]: %d 1/%d\n", z, 16383);
	tf_hal_printf("\n");
}

static TF_IMUV3 imu;

void example_setup(TF_HAL *hal) {
	// Create device object
	check(tf_imu_v3_create(&imu, UID, hal), "create device object");

	// Register quaternion callback to function quaternion_handler
	tf_imu_v3_register_quaternion_callback(&imu,
	                                       quaternion_handler,
	                                       NULL);

	// Set period for quaternion callback to 0.1s (100ms)
	tf_imu_v3_set_quaternion_callback_configuration(&imu, 100, false);
}

void example_loop(TF_HAL *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
