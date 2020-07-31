# NavUpdate Message

std_msgs/Header header

std_msgs/Bool isNavigating
std_msgs/Int32 curSegment
openpilot/Segment[] segments



# struct NavUpdate {
#   isNavigating @0 :Bool;
#   curSegment @1 :Int32;
#   segments @2 :List(Segment);

#   struct LatLng {
#     lat @0 :Float64;
#     lng @1 :Float64;
#   }

#   struct Segment {
#     from @0 :LatLng;
#     to @1 :LatLng;
#     updateTime @2 :Int32;
#     distance @3 :Int32;
#     crossTime @4 :Int32;
#     exitNo @5 :Int32;
#     instruction @6 :Instruction;

#     parts @7 :List(LatLng);

#     enum Instruction {
#       turnLeft @0;
#       turnRight @1;
#       keepLeft @2;
#       keepRight @3;
#       straight @4;
#       roundaboutExitNumber @5;
#       roundaboutExit @6;
#       roundaboutTurnLeft @7;
#       unkn8 @8;
#       roundaboutStraight @9;
#       unkn10 @10;
#       roundaboutTurnRight @11;
#       unkn12 @12;
#       roundaboutUturn @13;
#       unkn14 @14;
#       arrive @15;
#       exitLeft @16;
#       exitRight @17;
#       unkn18 @18;
#       uturn @19;
#       # ...
#     }
#   }
# }
