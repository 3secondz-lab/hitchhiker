# IonoData Message

std_msgs/UInt32 svHealth
std_msgs/Float64 tow
std_msgs/Float64 gpsWeek

std_msgs/Float64[] ionoAlpha
std_msgs/Float64[] ionoBeta

std_msgs/Bool healthValid
std_msgs/Bool ionoCoeffsValid



#  struct IonoData {
#    svHealth @0 :UInt32;
#    tow  @1 :Float64;
#    gpsWeek @2 :Float64;
#
#    ionoAlpha @3 :List(Float64);
#    ionoBeta @4 :List(Float64);
#
#    healthValid @5 :Bool;
#    ionoCoeffsValid @6 :Bool;
#  }
