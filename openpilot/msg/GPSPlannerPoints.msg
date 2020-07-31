# GPSPlannerPoints Message

std_msgs/Header header

openpilot/ECEFPoint curPos
openpilot/ECEFPoint[] points
std_msgs/Bool valid
std_msgs/String trackName
std_msgs/Float32 speedLimit
std_msgs/Float32 accelTarget



# struct GPSPlannerPoints {
#   curPosDEPRECATED @0 :ECEFPointDEPRECATED;
#   pointsDEPRECATED @1 :List(ECEFPointDEPRECATED);
#   curPos @6 :ECEFPoint;
#   points @7 :List(ECEFPoint);
#   valid @2 :Bool;
#   trackName @3 :Text;
#   speedLimit @4 :Float32;
#   accelTarget @5 :Float32;
# }
