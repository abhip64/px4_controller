# reference for polynomial trajectory tracking
#

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_SNAP_JERK = 1	# Position Velocity Acceleration Angular_Velocity Reference
uint8 IGNORE_SNAP_JERK_ANGVEL = 2	# Position Velocity Acceleration Reference
uint8 ACCEPT_SLITANG = 3      #Only accept slit angle
uint8 IGNORE_SNAP_JERK_ANGVEL_ACC = 4	# Position Velocity Reference

geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 ang_vel
float32 slit_ang
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 snap