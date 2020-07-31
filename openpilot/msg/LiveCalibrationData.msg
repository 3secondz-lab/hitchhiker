# LiveCalibrationData Message

std_msgs/Header header

# deprecated
std_msgs/Float32[] warpMatrix
# camera_frame_from_model_frame
std_msgs/Float32[] warpMatrix2
std_msgs/Float32[] warpMatrixBig
std_msgs/Int8 calStatus
std_msgs/Int32 calCycle
std_msgs/Int8 calPerc

# view_frame_from_road_frame
# ui's is inversed needs new
std_msgs/Float32[] extrinsicMatrix
# the direction of travel vector in device frame
std_msgs/Float32[] rpyCalib



# struct LiveCalibrationData {
#   # deprecated
#   warpMatrix @0 :List(Float32);
#   # camera_frame_from_model_frame
#   warpMatrix2 @5 :List(Float32);
#   warpMatrixBig @6 :List(Float32);
#   calStatus @1 :Int8;
#   calCycle @2 :Int32;
#   calPerc @3 :Int8;

#   # view_frame_from_road_frame
#   # ui's is inversed needs new
#   extrinsicMatrix @4 :List(Float32);
#   # the direction of travel vector in device frame
#   rpyCalib @7 :List(Float32);
# }
