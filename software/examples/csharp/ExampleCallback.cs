using System;
using Tinkerforge;

class Example
{
	private static string HOST = "localhost";
	private static int PORT = 4223;
	private static string UID = "XYZ"; // Change XYZ to the UID of your IMU Bricklet 3.0

	// Callback function for quaternion callback
	static void QuaternionCB(BrickletIMUV3 sender, short w, short x, short y, short z)
	{
		Console.WriteLine("Quaternion [W]: " + w/16383.0);
		Console.WriteLine("Quaternion [X]: " + x/16383.0);
		Console.WriteLine("Quaternion [Y]: " + y/16383.0);
		Console.WriteLine("Quaternion [Z]: " + z/16383.0);
		Console.WriteLine("");
	}

	static void Main()
	{
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletIMUV3 imu = new BrickletIMUV3(UID, ipcon); // Create device object

		ipcon.Connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Register quaternion callback to function QuaternionCB
		imu.QuaternionCallback += QuaternionCB;

		// Set period for quaternion callback to 0.1s (100ms)
		imu.SetQuaternionCallbackConfiguration(100, false);

		Console.WriteLine("Press enter to exit");
		Console.ReadLine();
		ipcon.Disconnect();
	}
}
