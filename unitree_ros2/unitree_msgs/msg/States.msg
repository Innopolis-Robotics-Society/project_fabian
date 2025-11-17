float64 t                          # Time 
float64[12] q_j                     # Joint position measurements
float64[12] v_j                     # Joint velocity measurements
float64[12] tau_j                   # Joint torque measurements
float64[4] imu_orientation         # Imu orientation estimation expressed as a quaternion (x, y, z, w)
float64[3] imu_angular_velocity    # Imu angular velocity
float64[3] imu_linear_acceleration # Imu linear acceleration 
float64[4] foot_force_sensor       # Force sensor measurements 