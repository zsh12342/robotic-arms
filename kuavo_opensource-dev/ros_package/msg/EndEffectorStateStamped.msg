Header header
string[] names
float64[] positions
float64[] velocities
float64[] accelerations 
float64[] torques
float64[] voltages

# if end_effector_type == JODELL, these status fields are available
# 0 : Not detected object
# 1 : Detected object while opening
# 2 : Detected object while closing
# 3 : Already reached the target position, but not detected object
float64[] status