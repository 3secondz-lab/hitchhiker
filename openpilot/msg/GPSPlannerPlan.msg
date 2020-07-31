# GPSPlannerPlan Message

std_msgs/Header header

std_msgs/Bool valid
std_msgs/Float32[] poly
std_msgs/String trackName
std_msgs/Float32 speed
std_msgs/Float32 acceleration
openpilot/ECEFPoint[] points
std_msgs/Float32 xLookahead



# struct GPSPlannerPlan {
#   valid @0 :Bool;
#   poly @1 :List(Float32);
#   trackName @2 :Text;
#   speed @3 :Float32;
#   acceleration @4 :Float32;
#   pointsDEPRECATED @5 :List(ECEFPointDEPRECATED);
#   points @6 :List(ECEFPoint);
#   xLookahead @7 :Float32;
# }
