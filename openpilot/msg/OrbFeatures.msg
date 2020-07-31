# OrbFeatures Message

std_msgs/Header header

std_msgs/UInt64 timestampEof
# transposed arrays of normalized image coordinates
# len(xs) == len(ys) == len(descriptors) * 32
std_msgs/Float32[] xs
std_msgs/Float32[] ys
std_msgs/String descriptors
std_msgs/Int8[] octaves

# match index to last OrbFeatures
# -1 if no match
std_msgs/UInt64 timestampLastEof
std_msgs/Int16[] matches



# struct OrbFeatures {
#   timestampEof @0 :UInt64;
#   # transposed arrays of normalized image coordinates
#   # len(xs) == len(ys) == len(descriptors) * 32
#   xs @1 :List(Float32);
#   ys @2 :List(Float32);
#   descriptors @3 :Data;
#   octaves @4 :List(Int8);

#   # match index to last OrbFeatures
#   # -1 if no match
#   timestampLastEof @5 :UInt64;
#   matches @6: List(Int16);
# }
