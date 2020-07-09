# MeasurementUbloxGnss Message

std_msgs/UInt8 svId
openpilot/TrackingStatus trackingStatus
# pseudorange in meters
std_msgs/Float64 pseudorange
# carrier phase measurement in cycles
std_msgs/Float64 carrierCycles
# doppler measurement in Hz
std_msgs/Float32 doppler
# GNSS id, 0 is gps
std_msgs/UInt8 gnssId
std_msgs/UInt8 glonassFrequencyIndex
# carrier phase locktime counter in ms
std_msgs/UInt16 locktime
# Carrier-to-noise density ratio (signal strength) in dBHz
std_msgs/UInt8 cno
# pseudorange standard deviation in meters
std_msgs/Float32 pseudorangeStdev
# carrier phase standard deviation in cycles
std_msgs/Float32 carrierPhaseStdev
# doppler standard deviation in Hz
std_msgs/Float32 dopplerStdev
std_msgs/UInt8 sigId
