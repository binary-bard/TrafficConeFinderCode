# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# Since we are dealing with images and we might have real distance sensors 
# in case of some cameras like realsense, this flag indicates whether distance
# is real or in pixels
bool distance_is_real

# Use Pose2D for the distance - we will use a list later
geometry_msgs/Pose2D[] poses