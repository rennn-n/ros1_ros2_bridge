int32 gid # Global ID
int32 lid # Local ID
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
DTLane dtlane # Waypoint in the middle of the lane
int32 change_flag # Flag for lane change
WaypointState wpstate # What makes waypoints read event flags (for vectormap)
uint32 lane_id # Motal vehicle lane number
uint32 left_lane_id # Left lane number
uint32 right_lane_id # Right lane number
uint32 stop_line_id # Stop the identifier for a while
float32 cost
float32 time_cost
# Lane Direction
# FORWARD        = 0
# FORWARD_LEFT       = 1
# FORWARD_RIGHT      = 2
# BACKWARD        = 3 
# BACKWARD_LEFT      = 4
# BACKWARD_RIGHT    = 5
# STANDSTILL       = 6
uint32 direction