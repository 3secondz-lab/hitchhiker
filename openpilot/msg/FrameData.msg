# FrameData Message

std_msgs/Header header

std_msgs/UInt32 frameId
std_msgs/UInt32 encodeId
std_msgs/UInt64 timestampEof
std_msgs/Int32 frameLength
std_msgs/Int32 integLines
std_msgs/Int32 globalGain
std_msgs/Int32 lensPos
std_msgs/Float32 lensSag
std_msgs/Float32 lensErr
std_msgs/Float32 lensTruePos
std_msgs/String image
std_msgs/Float32 gainFrac
std_msgs/Int16[] focusVal
std_msgs/UInt8[] focusConf
std_msgs/UInt16[] sharpnessScore
std_msgs/Int32 recoverState

std_msgs/UInt64 frameType
std_msgs/UInt64 timestampSof
std_msgs/Float32[] transform

openpilot/AndroidCaptureResult AndroidCaptureResult



# struct FrameData {
#   frameId @0 :UInt32;
#   encodeId @1 :UInt32; # DEPRECATED
#   timestampEof @2 :UInt64;
#   frameLength @3 :Int32;
#   integLines @4 :Int32;
#   globalGain @5 :Int32;
#   lensPos @11 :Int32;
#   lensSag @12 :Float32;
#   lensErr @13 :Float32;
#   lensTruePos @14 :Float32;
#   image @6 :Data;
#   gainFrac @15 :Float32;
#   focusVal @16 :List(Int16);
#   focusConf @17 :List(UInt8);
#   sharpnessScore @18 :List(UInt16);
#   recoverState @19 :Int32;

#   frameType @7 :FrameType;
#   timestampSof @8 :UInt64;
#   transform @10 :List(Float32);

#   androidCaptureResult @9 :AndroidCaptureResult;

#   enum FrameType {
#     unknown @0;
#     neo @1;
#     chffrAndroid @2;
#     front @3;
#   }

#   struct AndroidCaptureResult {
#     sensitivity @0 :Int32;
#     frameDuration @1 :Int64;
#     exposureTime @2 :Int64;
#     rollingShutterSkew @3 :UInt64;
#     colorCorrectionTransform @4 :List(Int32);
#     colorCorrectionGains @5 :List(Float32);
#     displayRotation @6 :Int8;
#   }
# }
