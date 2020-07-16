# DriverState Message

std_msgs/Header header

std_msgs/UInt32 frameId
std_msgs/Float32[] faceOrientation
std_msgs/Float32[] facePosition
std_msgs/Float32 faceProb
std_msgs/Float32 leftEyeProb
std_msgs/Float32 rightEyeProb
std_msgs/Float32 leftBlinkProb
std_msgs/Float32 rightBlinkProb
std_msgs/Float32[] faceOrientationStd
std_msgs/Float32[] facePositionStd
std_msgs/Float32 sgProb



# struct DriverState {
#   frameId @0 :UInt32;
#   descriptorDEPRECATED @1 :List(Float32);
#   stdDEPRECATED @2 :Float32;
#   faceOrientation @3 :List(Float32);
#   facePosition @4 :List(Float32);
#   faceProb @5 :Float32;
#   leftEyeProb @6 :Float32;
#   rightEyeProb @7 :Float32;
#   leftBlinkProb @8 :Float32;
#   rightBlinkProb @9 :Float32;
#   irPwrDEPRECATED @10 :Float32;
#   faceOrientationStd @11 :List(Float32);
#   facePositionStd @12 :List(Float32);
#   sgProb @13 :Float32;
# }
