# RadarPoint Message

std_msgs/UInt64 trackId

std_msgs/Float32 dRel
std_msgs/Float32 yRel
std_msgs/Float32 vRel

std_msgs/Float32 aRel
std_msgs/Float32 yvRel

std_msgs/Bool measured



#  struct RadarPoint {
#    trackId @0 :UInt64;  # no trackId reuse
#
#    # these 3 are the minimum required
#    dRel @1 :Float32; # m from the front bumper of the car
#    yRel @2 :Float32; # m
#    vRel @3 :Float32; # m/s
#
#    # these are optional and valid if they are not NaN
#    aRel @4 :Float32; # m/s^2
#    yvRel @5 :Float32; # m/s
#
#    # some radars flag measurements VS estimates
#    measured @6 :Bool;
#  }
