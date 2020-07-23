# UiNavigationEvent Message

std_msgs/Header header

std_msgs/String type
std_msgs/String status
std_msgs/Float32 distanceTo
openpilot/ECEFPoint endRoadPoint



# struct UiNavigationEvent {
#   type @0: Type;
#   status @1: Status;
#   distanceTo @2: Float32;
#   endRoadPointDEPRECATED @3: ECEFPointDEPRECATED;
#   endRoadPoint @4: ECEFPoint;

#   enum Type {
#     none @0;
#     laneChangeLeft @1;
#     laneChangeRight @2;
#     mergeLeft @3;
#     mergeRight @4;
#     turnLeft @5;
#     turnRight @6;
#   }

#   enum Status {
#     none @0;
#     passive @1;
#     approaching @2;
#     active @3;
#   }
# }
