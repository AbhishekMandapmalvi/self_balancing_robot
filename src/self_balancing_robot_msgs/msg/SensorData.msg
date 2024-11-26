# Single message combining IMU and encoder data
Header header

# IMU data in Euler angles (degrees)
float32 roll
float32 pitch
float32 yaw

# Angular velocities (rad/s)
float32 gyro_x
float32 gyro_y
float32 gyro_z

# Linear accelerations (m/s^2)
float32 accel_x
float32 accel_y
float32 accel_z

# Encoder counts
int32 left_count
int32 right_count

# Calibration status
uint8 system_cal
uint8 gyro_cal
uint8 accel_cal
uint8 mag_cal