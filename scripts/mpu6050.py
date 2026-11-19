from machine import Pin, I2C
from time import sleep


class MPU6050:
    def __init__(self, scl_id=21, sda_id=20, i2c_addr=0x68, dlpf_level=4):
        # Variable
        self.lin_acc_x = 0.0
        self.lin_acc_y = 0.0
        self.lin_acc_z = 0.0
        self.ang_vel_x = 0.0
        self.ang_vel_y = 0.0
        self.ang_vel_z = 0.0
        # Constants
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        # Config IMU
        self.i2c = I2C(0, scl=Pin(scl_id), sda=Pin(sda_id), freq=400_000)
        self.address = i2c_addr
        self.i2c.writeto_mem(i2c_addr, 0x6B, bytes([0x00]))
        self.i2c.writeto_mem(i2c_addr, 0x1A, bytes([dlpf_level]))
        # Calibrate Gyroscope
        # sleep(3)  # hands off time
        self.calibrate_gyro()
        
    def read_data(self):
        def process_raw(data, scale):
            if data > 2**15:
                value = (data - 2**16) / scale
            else:
                value = data / scale
            return value
        words = self.i2c.readfrom_mem(self.address, 0x3B, 14)
        data = [words[i] << 8 | words[i] for i in range(0, len(words), 2)]
        self.lin_acc_x = process_raw(data[0], 16384) * 9.81
        self.lin_acc_y = process_raw(data[1], 16384) * 9.81
        self.lin_acc_z = process_raw(data[2], 16384) * 9.81
        self.ang_vel_x = process_raw(data[-3], 131) - self.gyro_bias_x
        self.ang_vel_y = process_raw(data[-2], 131) - self.gyro_bias_y
        self.ang_vel_z = process_raw(data[-1], 131) - self.gyro_bias_z
        
        return {
            "acc_x": self.lin_acc_x,
            "acc_y": self.lin_acc_y,
            "acc_z": self.lin_acc_z,
            "omg_x": self.ang_vel_x,
            "omg_y": self.ang_vel_y,
            "omg_z": self.ang_vel_z,
        }
        
    def calibrate_gyro(self, num_samples=1000):
        omg_x_accumulation = 0.0
        omg_y_accumulation = 0.0
        omg_z_accumulation = 0.0
        for i in range(num_samples):
            data = self.read_data()
            omg_x_accumulation = omg_x_accumulation + data["omg_x"]
            omg_y_accumulation = omg_y_accumulation + data["omg_y"]
            omg_z_accumulation += data["omg_z"]
            sleep(0.005)
        self.gyro_bias_x = omg_x_accumulation / num_samples
        self.gyro_bias_y = omg_y_accumulation / num_samples
        self.gyro_bias_z = omg_z_accumulation / num_samples
        
            

if __name__ == "__main__":
    imu = MPU6050(scl_id=21, sda_id=20, i2c_addr=0x68, dlpf_level=2)
    while True:
        motion_data = imu.read_data()
        print(motion_data)
        sleep(0.1)
        
