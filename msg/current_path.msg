# Current path output from the path manager or received from HIL simulator */
uint64 timestamp	# in microseconds since system start
# @warning Va_d have always to be valid, r and q need to be valid if flag is true, c rho and lambda need to be valid otherwise
bool flag		# Inicates strait line or orbital path (true is line, false is orbit)
float32 Va_d		# Desired airspeed (m/s)
float32[3] r		# Vector to origin of straight line path (m)
float32[3] q		# Unit vector, desired direction of travel for line path
float32[3] c		# Center of orbital path (m)
float32 rho		# Radius of orbital path (m)
int8 lambda		# Direction of orbital path (clockwise is 1, counterclockwise is -1)
