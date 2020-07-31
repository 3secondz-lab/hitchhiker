# CameraOdometry Message

std_msgs/Header header

std_msgs/UInt32 frameId
std_msgs/UInt64 timestampEof
std_msgs/Float32[] trans
std_msgs/Float32[] rot
std_msgs/Float32[] transStd
std_msgs/Float32[] rotStd



# struct CameraOdometry {
#   frameId @4 :UInt32;
#   timestampEof @5 :UInt64;
#   trans @0 :List(Float32); # m/s in device frame
#   rot @1 :List(Float32); # rad/s in device frame
#   transStd @2 :List(Float32); # std m/s in device frame
#   rotStd @3 :List(Float32); # std rad/s in device frame
# }
