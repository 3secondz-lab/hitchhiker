# TrafficEvent Message

std_msgs/Header header

std_msgs/UInt64 type
std_msgs/Float32 distance
std_msgs/UInt64 action
std_msgs/Bool resuming



# struct TrafficEvent @0xacfa74a094e62626 {
#   type @0 :Type;
#   distance @1 :Float32;
#   action @2 :Action;
#   resuming @3 :Bool;

#   enum Type {
#     stopSign @0;
#     lightRed @1;
#     lightYellow @2;
#     lightGreen @3;
#     stopLight @4;
#   }

#   enum Action {
#     none @0;
#     yield @1;
#     stop @2;
#     resumeReady @3;
#   }

# }
