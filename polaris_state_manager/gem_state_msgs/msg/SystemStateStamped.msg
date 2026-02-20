std_msgs/Header header

# The following error codes represent bit masks
# We can encode one or more of these in the error_code field
uint8 ERROR_NONE=0
uint8 ERROR_BATTERY_LOW=1
uint8 ERROR_ESTOP=2
uint8 ERROR_GPS_LOST=4
uint8 ERROR_SIGNAL_LOST=8
uint8 ERROR_SIGNAL_LOW=16
uint8 ERROR_TEMPERATURE_HIGH=32
# This is a generic stale data error if any of the input streams
# become stale
uint8 ERROR_STALE_DATA=64

# The system can be in one of the following states
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_ERROR=2

uint8 error_code
uint8 state

