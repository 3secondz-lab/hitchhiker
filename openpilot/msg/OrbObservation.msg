# OrbObservation Message

std_msgs/Header header

std_msgs/UInt64 observationMonoTime
std_msgs/Float32[] normalizedCoordinates
std_msgs/Float64[] locationECEF
std_msgs/UInt32 matchDistance



# struct OrbObservation {
#   observationMonoTime @0 :UInt64;
#   normalizedCoordinates @1 :List(Float32);
#   locationECEF @2 :List(Float64);
#   matchDistance @3: UInt32;
# }
