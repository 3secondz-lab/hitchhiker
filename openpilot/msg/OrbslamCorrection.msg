# OrbslamCorrection Message

std_msgs/Header header

std_msgs/UInt64 correctionMonoTime
std_msgs/Float64[] prePositionECEF
std_msgs/Float64[] postPositionECEF
std_msgs/Float32[] prePoseQuatECEF
std_msgs/Float32[] postPoseQuatECEF
std_msgs/UInt32 numInliers



# struct OrbslamCorrection {
#   correctionMonoTime @0 :UInt64;
#   prePositionECEF @1 :List(Float64);
#   postPositionECEF @2 :List(Float64);
#   prePoseQuatECEF @3 :List(Float32);
#   postPoseQuatECEF @4 :List(Float32);
#   numInliers @5 :UInt32;
# }
