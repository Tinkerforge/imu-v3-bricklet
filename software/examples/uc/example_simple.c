// This example is not self-contained.
// It requires usage of the example driver specific to your platform.
// See the HAL documentation.

#include "src/bindings/hal_common.h"
#include "src/bindings/bricklet_imu_v3.h"

void check(int rc, const char *msg);
void example_setup(TF_HAL *hal);
void example_loop(TF_HAL *hal);

static TF_IMUV3 imu;

void example_setup(TF_HAL *hal) {
	// Create device object
	check(tf_imu_v3_create(&imu, NULL, hal), "create device object");

	// Get current quaternion
	int16_t w, x, y, z;
	check(tf_imu_v3_get_quaternion(&imu, &w, &x, &y, &z), "get quaternion");

	tf_hal_printf("Quaternion [W]: %d 1/%d\n", w, 16383);
	tf_hal_printf("Quaternion [X]: %d 1/%d\n", x, 16383);
	tf_hal_printf("Quaternion [Y]: %d 1/%d\n", y, 16383);
	tf_hal_printf("Quaternion [Z]: %d 1/%d\n", z, 16383);
}

void example_loop(TF_HAL *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
