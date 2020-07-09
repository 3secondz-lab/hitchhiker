# MeasurementReportUbloxGnss Message

#received time of week in gps time in seconds and gps week
std_msgs/Float64 rcvTow
std_msgs/UInt16 gpsWeek
# leap seconds in seconds
std_msgs/UInt16 leapSeconds
# receiver status
openpilot/ReceiverStatus receiverStatus
# num of measurements to follow
std_msgs/UInt8 numMeas
openpilot/MeasurementUbloxGnss[] measurements
