import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickletIMUV3;
import com.tinkerforge.BrickletIMUV3.Quaternion;

public class ExampleSimple {
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

		// Get current quaternion
		Quaternion quaternion = imu.getQuaternion(); // Can throw com.tinkerforge.TimeoutException

		System.out.println("Quaternion [W]: " + quaternion.w/16383.0);
		System.out.println("Quaternion [X]: " + quaternion.x/16383.0);
		System.out.println("Quaternion [Y]: " + quaternion.y/16383.0);
		System.out.println("Quaternion [Z]: " + quaternion.z/16383.0);

		System.out.println("Press key to exit"); System.in.read();
		ipcon.disconnect();
	}
}
