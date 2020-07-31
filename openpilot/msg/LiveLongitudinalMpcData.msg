# LiveLongitudinalMpcData Message

std_msgs/Header header

std_msgs/Float32[] xEgo
std_msgs/Float32[] vEgo
std_msgs/Float32[] aEgo
std_msgs/Float32[] xLead
std_msgs/Float32[] vLead
std_msgs/Float32[] aLead
std_msgs/Float32 aLeadTau
std_msgs/UInt32 qpIterations
std_msgs/UInt32 mpcId
std_msgs/UInt64 calculationTime
std_msgs/Float64 cost



# struct LiveLongitudinalMpcData {
#   xEgo @0 :List(Float32);
#   vEgo @1 :List(Float32);
#   aEgo @2 :List(Float32);
#   xLead @3 :List(Float32);
#   vLead @4 :List(Float32);
#   aLead @5 :List(Float32);
#   aLeadTau @6 :Float32;    # lead accel time constant
#   qpIterations @7 :UInt32;
#   mpcId @8 :UInt32;
#   calculationTime @9 :UInt64;
#   cost @10 :Float64;
# }
