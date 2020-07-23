# RadarData Message

std_msgs/Header header

std_msgs/String[] errors
openpilot/RadarPoint[] points

# which packets this state came from
std_msgs/UInt64[] canMonoTimes



# struct RadarData @0x888ad6581cf0aacb {
#   errors @0 :List(Error);
#   points @1 :List(RadarPoint);

#   # which packets this state came from
#   canMonoTimes @2 :List(UInt64);

#   enum Error {
#     canError @0;
#     fault @1;
#     wrongConfig @2;
#   }

#   # similar to LiveTracks
#   # is one timestamp valid for all? I think so
#   struct RadarPoint {
#     trackId @0 :UInt64;  # no trackId reuse

#     # these 3 are the minimum required
#     dRel @1 :Float32; # m from the front bumper of the car
#     yRel @2 :Float32; # m
#     vRel @3 :Float32; # m/s

#     # these are optional and valid if they are not NaN
#     aRel @4 :Float32; # m/s^2
#     yvRel @5 :Float32; # m/s

#     # some radars flag measurements VS estimates
#     measured @6 :Bool;
#   }
# }
