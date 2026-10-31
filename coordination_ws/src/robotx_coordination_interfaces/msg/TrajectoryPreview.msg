# Samples correspond to header.stamp + offset_seconds, always 0, dt, 2*dt.
# Consumers MUST stop using an expired, invalid or superseded run reference.
std_msgs/Header header
string reference_id
string run_id
string phase
bool valid
string reason
float64 dt
TrajectorySample[] samples
