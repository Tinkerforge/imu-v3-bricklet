import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickletIMUV3;

public class ExampleCallback {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XYZ to the UID of your IMU Bricklet 3.0
	private static final String UID = "XYZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletIMUV3 imu = new BrickletIMUV3(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Add quaternion listener
		imu.addQuaternionListener(new BrickletIMUV3.QuaternionListener() {
			public void quaternion(int w, int x, int y, int z) {
				System.out.println("Quaternion [W]: " + w/16383.0);
				System.out.println("Quaternion [X]: " + x/16383.0);
				System.out.println("Quaternion [Y]: " + y/16383.0);
				System.out.println("Quaternion [Z]: " + z/16383.0);
				System.out.println("");
			}
		});

		// Set period for quaternion callback to 0.1s (100ms)
		imu.setQuaternionCallbackConfiguration(100, false);

		System.out.println("Press key to exit"); System.in.read();
		ipcon.disconnect();
	}
}
