use std::{error::Error, io, thread};
use tinkerforge::{imu_v3_bricklet::*, ip_connection::IpConnection};

const HOST: &str = "localhost";
const PORT: u16 = 4223;
const UID: &str = "XYZ"; // Change XYZ to the UID of your IMU Bricklet 3.0.

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection.
    let imu = ImuV3Bricklet::new(UID, &ipcon); // Create device object.

    ipcon.connect((HOST, PORT)).recv()??; // Connect to brickd.
                                          // Don't use device before ipcon is connected.

    let quaternion_receiver = imu.get_quaternion_callback_receiver();

    // Spawn thread to handle received callback messages.
    // This thread ends when the `imu` object
    // is dropped, so there is no need for manual cleanup.
    thread::spawn(move || {
        for quaternion in quaternion_receiver {
            println!("Quaternion [W]: {}", quaternion.w as f32 / 16383.0);
            println!("Quaternion [X]: {}", quaternion.x as f32 / 16383.0);
            println!("Quaternion [Y]: {}", quaternion.y as f32 / 16383.0);
            println!("Quaternion [Z]: {}", quaternion.z as f32 / 16383.0);
            println!();
        }
    });

    // Set period for quaternion callback to 0.1s (100ms).
    imu.set_quaternion_callback_configuration(100, false);

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    ipcon.disconnect();
    Ok(())
}
