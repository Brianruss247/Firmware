# Controller commands output from the path follower or received from HIL simulator */
uint64 timestamp	# in microseconds since system start
# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional
float32 Va_c		# Commanded airspeed (m/s)
float32 h_c		# Commanded altitude (m)
float32 chi_c		# Commanded course (rad)
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid
