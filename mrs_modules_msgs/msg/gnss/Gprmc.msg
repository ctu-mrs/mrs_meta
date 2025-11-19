# Message from GPRMC NMEA String
std_msgs/Header header

string message_id

float64 utc_seconds
GpsStatus position_status

float64 latitude
float64 longitude

string latitude_dir
string longitude_dir

float32 speed  # in metres per second
float32 track
string date
float32 mag_var
string mag_var_direction

string mode_indicator
string MODE_INDICATIOR_AUTONOMOUS=a
string MODE_INDICATIOR_DIFFERENTIAL=d
string MODE_INDICATIOR_ESTIMATED=e
string MODE_INDICATIOR_MANUAL=m
string MODE_INDICATIOR_DATA_NOT_VALID=n
