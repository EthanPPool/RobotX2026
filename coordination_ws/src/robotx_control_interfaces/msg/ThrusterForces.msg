# Signed model forces: Ft3 = RIGHT/starboard, Ft4 = LEFT/port.
# Positive force direction and SI unit/axis binding MUST be confirmed separately.
# A valid shadow command alone does not grant propulsion authority.
std_msgs/Header header
builtin_interfaces/Time valid_until
string reference_id
string run_id
string controller_id
uint64 sequence
bool valid
bool model_contract_confirmed
string reason
float64 right_force_n
float64 left_force_n
