# EncodeIndex Message

std_msgs/Header header

# picture from camera
std_msgs/UInt32 frameId
std_msgs/String type
# index of encoder from start of route
std_msgs/UInt32 encodeId
# minute long segment this frame is in
std_msgs/Int32 segmentNum
# index into camera file in segment in presentation order
std_msgs/UInt32 segmentId
# index into camera file in segment in encode order
std_msgs/UInt32 segmentIdEncode



# struct EncodeIndex {
#   # picture from camera
#   frameId @0 :UInt32;
#   type @1 :Type;
#   # index of encoder from start of route
#   encodeId @2 :UInt32;
#   # minute long segment this frame is in
#   segmentNum @3 :Int32;
#   # index into camera file in segment in presentation order
#   segmentId @4 :UInt32;
#   # index into camera file in segment in encode order
#   segmentIdEncode @5 :UInt32;

#   enum Type {
#     bigBoxLossless @0;   # rcamera.mkv
#     fullHEVC @1;         # fcamera.hevc
#     bigBoxHEVC @2;       # bcamera.hevc
#     chffrAndroidH264 @3; # acamera
#     fullLosslessClip @4; # prcamera.mkv
#     front @5;            # dcamera.hevc
#   }
# }
