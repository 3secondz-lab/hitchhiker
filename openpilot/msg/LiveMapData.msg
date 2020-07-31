# LiveMapData Message

std_msgs/Header header

std_msgs/Bool speedLimitValid
std_msgs/Float32 speedLimit
std_msgs/Bool speedAdvisoryValid
std_msgs/Float32 speedAdvisory
std_msgs/Bool speedLimitAheadValid
std_msgs/Float32 speedLimitAhead
std_msgs/Float32 speedLimitAheadDistance
std_msgs/Bool curvatureValid
std_msgs/Float32 curvature
std_msgs/UInt64 wayId
std_msgs/Float32[] roadX
std_msgs/Float32[] roadY
openpilot/GpsLocationData lastGps
std_msgs/Float32[] roadCurvatureX
std_msgs/Float32[] roadCurvature
std_msgs/Float32 distToTurn
std_msgs/Bool mapValid



# struct LiveMapData {
#   speedLimitValid @0 :Bool;
#   speedLimit @1 :Float32;
#   speedAdvisoryValid @12 :Bool;
#   speedAdvisory @13 :Float32;
#   speedLimitAheadValid @14 :Bool;
#   speedLimitAhead @15 :Float32;
#   speedLimitAheadDistance @16 :Float32;
#   curvatureValid @2 :Bool;
#   curvature @3 :Float32;
#   wayId @4 :UInt64;
#   roadX @5 :List(Float32);
#   roadY @6 :List(Float32);
#   lastGps @7: GpsLocationData;
#   roadCurvatureX @8 :List(Float32);
#   roadCurvature @9 :List(Float32);
#   distToTurn @10 :Float32;
#   mapValid @11 :Bool;
# }
