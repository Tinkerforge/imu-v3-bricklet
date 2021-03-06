// This example is not self-contained.
// It requres usage of the example driver specific to your platform.
// See the HAL documentation.

#include "bindings/hal_common.h"
#include "bindings/bricklet_imu_v3.h"

#define UID "XYZ" // Change XYZ to the UID of your IMU Bricklet 3.0

void check(int rc, const char* msg);

void example_setup(TF_HalContext *hal);
void example_loop(TF_HalContext *hal);


static TF_IMUV3 imu;

void example_setup(TF_HalContext *hal) {
	// Create device object
	check(tf_imu_v3_create(&imu, UID, hal), "create device object");

	// Get current quaternion
	int16_t w, x, y, z;
	check(tf_imu_v3_get_quaternion(&imu, &w, &x, &y, &z), "get quaternion");

	tf_hal_printf("Quaternion [W]: %d 1/%d\n", w, 16383);
	tf_hal_printf("Quaternion [X]: %d 1/%d\n", x, 16383);
	tf_hal_printf("Quaternion [Y]: %d 1/%d\n", y, 16383);
	tf_hal_printf("Quaternion [Z]: %d 1/%d\n", z, 16383);
}

void example_loop(TF_HalContext *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
