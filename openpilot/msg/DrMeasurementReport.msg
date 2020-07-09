# DrMeasurementReport Message

std_msgs/UInt8 reason
std_msgs/UInt8 seqNum
std_msgs/UInt8 seqMax
std_msgs/UInt16 rfLoss

std_msgs/Bool systemRtcValid
std_msgs/UInt32 fCount
std_msgs/UInt32 clockResets
std_msgs/UInt64 systemRtcTime

std_msgs/UInt8 gpsLeapSeconds
std_msgs/UInt8 gpsLeapSecondsUncertainty
std_msgs/Float32 gpsToGlonassTimeBiasMilliseconds
std_msgs/Float32 gpsToGlonassTimeBiasMillisecondsUncertainty

std_msgs/UInt16 gpsWeek
std_msgs/UInt32 gpsMilliseconds
std_msgs/UInt32 gpsTimeBiasMs
std_msgs/UInt32 gpsClockTimeUncertaintyMs
std_msgs/UInt8 gpsClockSource

std_msgs/UInt8 glonassClockSource
std_msgs/UInt8 glonassYear
std_msgs/UInt16 glonassDay
std_msgs/UInt32 glonassMilliseconds
std_msgs/Float32 glonassTimeBias
std_msgs/Float32 glonassClockTimeUncertainty

std_msgs/Float32 clockFrequencyBias
std_msgs/Float32 clockFrequencyUncertainty
std_msgs/UInt8 frequencySource

std_msgs/UInt64 source

openpilot/SVDrMeasurementReport[] sv
