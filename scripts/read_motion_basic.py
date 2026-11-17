from machine import Pin, I2C
from math import pi
from time import sleep

# SETUP
# Variables
lin_acc_x = 0.0
lin_acc_y = 0.0
lin_acc_z = 0.0
ang_vel_x = 0.0
ang_vel_y = 0.0
ang_vel_z = 0.0
# Instantiate an I2C communicator
imu = I2C(0, scl=Pin(21), sda=Pin(20), freq=400_000)
imu.writeto_mem(
    0x68, 0x6B, bytes([0x00])
)  # write value: 0 to sensor's power management register: 0x6B to wake up the sensor, which located at address: 0x68 on the I2C BUS.
imu.writeto_mem(
    0x68, 0x1A, bytes([3])
)  ## write value: 3 to sensor's register: 0x1A to filter out high frequency noise, level 3 ~= 44 Hz, of the sensor, which located at address: 0x68 on the I2C BUS.


# Define a helper function to process raw data
def process_raw(data, scale):
    """
    Args:
        data: an unsigned 16-bit of sensor data
        scale: a constant coefficient scaling raw data.
                accelerometer: 16384 per g
                gyro: 131 per deg/s,
    Returns:
        value: human readible value in m/s^2 or deg/s
    """
    if data > 32768:  # convert to signed int16
        value = (data - 65535) / scale
    else:
        value = data / scale

    return value


# LOOP
while True:
    words = imu.readfrom_mem(
        0x68,
        0x3B,  # ACCEL_XOUT_H register address
        14,  # number of bytes
    )  # retrieve raw sensor data in bytes
    # Preprocess bytes, split 2 bytes as a group
    data = [words[i] << 8 | words[i + 1] for i in range(0, len(words), 2)]
    # Calculate human readibles
    lin_acc_x = process_raw(data[0], 16384) * 9.80665
    lin_acc_y = process_raw(data[1], 16384) * 9.80665
    lin_acc_z = process_raw(data[2], 16384) * 9.80665
    ang_vel_x = process_raw(data[4], 131) * pi / 180
    ang_vel_y = process_raw(data[5], 131) * pi / 180
    ang_vel_z = process_raw(data[6], 131) * pi / 180
    print("Motion data:")
    print(
        f"linear acceleration on X: {lin_acc_x:.2f}, Y: {lin_acc_y:.2f}, Z: {lin_acc_z:.2f}"
    )
    print(
        f"angular velocity on X: {ang_vel_x:.2f}, Y: {ang_vel_y:.2f}, Z: {ang_vel_z:.2f}"
    )
    sleep(0.1)
