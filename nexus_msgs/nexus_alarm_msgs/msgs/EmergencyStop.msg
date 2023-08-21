# This message holds the emergency stop (EMStop) status of the system.
# It detects an EMStop caused by the emergency stop buttons.

std_msgs/Header header

# Possible EMStop States
int16 STATE_INACTIVE = 0   # system operating normal
int16 STATE_ACTIVE = 1     # emergency stop is active (Button pressed)
string message             # Description message, empty if the system is working
                           # properly and with some string message when the
                           # system is in an emergency stop

bool emergency_button_stop # true = emergency stop signal is issued by button
                           # pressed
int16 emergency_state      # state (including confimation by key-switch),
                           # values see above
