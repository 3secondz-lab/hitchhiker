# DMonitoringState Message

std_msgs/Header header

# TODO: deprecate old fields in controlsState
openpilot/CarEvent[] events
std_msgs/Bool faceDetected
std_msgs/Bool isDistracted
std_msgs/Float32 awarenessStatus
std_msgs/Bool isRHD
std_msgs/Bool rhdChecked
std_msgs/Float32 posePitchOffset
std_msgs/UInt32 posePitchValidCount
std_msgs/Float32 poseYawOffset
std_msgs/UInt32 poseYawValidCount
std_msgs/Float32 stepChange
std_msgs/Float32 awarenessActive
std_msgs/Float32 awarenessPassive
std_msgs/Bool isLowStd
std_msgs/UInt32 hiStdCount
std_msgs/Bool isPreview



# struct DMonitoringState {
#   # TODO: deprecate old fields in controlsState
#   events @0 :List(Car.CarEvent);
#   faceDetected @1 :Bool;
#   isDistracted @2 :Bool;
#   awarenessStatus @3 :Float32;
#   isRHD @4 :Bool;
#   rhdChecked @5 :Bool;
#   posePitchOffset @6 :Float32;
#   posePitchValidCount @7 :UInt32;
#   poseYawOffset @8 :Float32;
#   poseYawValidCount @9 :UInt32;
#   stepChange @10 :Float32;
#   awarenessActive @11 :Float32;
#   awarenessPassive @12 :Float32;
#   isLowStd @13 :Bool;
#   hiStdCount @14 :UInt32;
#   isPreview @15 :Bool;
# }
