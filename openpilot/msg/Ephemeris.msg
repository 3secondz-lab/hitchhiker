# Ephemeris Message

# This is according to the rinex (2?) format
std_msgs/UInt16 svId
std_msgs/UInt16 year
std_msgs/UInt16 month
std_msgs/UInt16 day
std_msgs/UInt16 hour
std_msgs/UInt16 minute
std_msgs/Float32 second
std_msgs/Float64 af0
std_msgs/Float64 af1
std_msgs/Float64 af2

std_msgs/Float64 iode
std_msgs/Float64 crs
std_msgs/Float64 deltaN
std_msgs/Float64 m0

std_msgs/Float64 cuc
std_msgs/Float64 ecc
std_msgs/Float64 cus
std_msgs/Float64 a

std_msgs/Float64 toe
std_msgs/Float64 cic
std_msgs/Float64 omega0
std_msgs/Float64 cis

std_msgs/Float64 i0
std_msgs/Float64 crc
std_msgs/Float64 omega
std_msgs/Float64 omegaDot

std_msgs/Float64 iDot
std_msgs/Float64 codesL2
std_msgs/Float64 gpsWeek
std_msgs/Float64 l2

std_msgs/Float64 svAcc
std_msgs/Float64 svHealth
std_msgs/Float64 tgd
std_msgs/Float64 iodc

std_msgs/Float64 transmissionTime
std_msgs/Float64 fitInterval

std_msgs/Float64 toc

std_msgs/Bool ionoCoeffsValid
std_msgs/Float64[] ionoAlpha
std_msgs/Float64[] ionoBeta
