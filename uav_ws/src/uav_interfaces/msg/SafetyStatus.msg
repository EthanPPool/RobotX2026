builtin_interfaces/Time stamp

uint8 INIT=0
uint8 NOT_READY=1
uint8 READY=2
uint8 ACTIVE=3
uint8 FAILSAFE=4

uint8 state
string state_label
bool ready
bool prearm_ready
bool flight_ready
bool failsafe_latched
bool autonomy_enabled

bool mavros_state_fresh
bool mavros_connected
bool armed
bool mode_allowed
bool gps_valid
bool local_position_valid
bool battery_valid
float32 battery_percentage
bool mission_healthy

string reason
