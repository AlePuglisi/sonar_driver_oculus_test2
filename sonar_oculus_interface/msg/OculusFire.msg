# Map the struct OculusSimpleFireMessage in Oculus.h
std_msgs/Header header

uint8    mode               # Operation mode (simple, high-res, etc.)
uint8    gamma              # Image gamma correction
uint8    flags              # Configuration bits (auto-gain, data compression, etc.)
float64  range              # Maximum range (unit of measure depends on flags) 
float64  gain               # Sonar gain level
float64  speed_of_sound     # Used to compute ranges [m/s] if 0 it is based on salinity 
float64  salinity           # Used for sound speed correction