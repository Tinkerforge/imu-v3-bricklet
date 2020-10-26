function octave_example_simple()
    more off;

    HOST = "localhost";
    PORT = 4223;
    UID = "XYZ"; % Change XYZ to the UID of your IMU Bricklet 3.0

    ipcon = javaObject("com.tinkerforge.IPConnection"); % Create IP connection
    imu = javaObject("com.tinkerforge.BrickletIMUV3", UID, ipcon); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Get current quaternion
    quaternion = imu.getQuaternion();

    fprintf("Quaternion [W]: %g\n", quaternion.w/16383.0);
    fprintf("Quaternion [X]: %g\n", quaternion.x/16383.0);
    fprintf("Quaternion [Y]: %g\n", quaternion.y/16383.0);
    fprintf("Quaternion [Z]: %g\n", quaternion.z/16383.0);

    input("Press key to exit\n", "s");
    ipcon.disconnect();
end
