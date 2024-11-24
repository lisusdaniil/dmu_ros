# Augmented IMU data from DMU sensor. Counter is particularly
# important for re-sampling of data, since otherwise dependent on
# unsynchronized ROS time.

# header.stamp specifies the ROS time for this measurement
#
# header.frame_id is the frame of reference

Header header

# Number of message
int32 msg_count

# IMU values
sensor_msgs/Imu imu

# Average Imu temperature
float64 average_imu_temp

# System flags
int16 system_startup_flags
int16 system_operat_flags