# PathPlan Message

std_msgs/Header header

std_msgs/Float32 laneWidth

std_msgs/Float32[] dPoly
std_msgs/Float32[] cPoly
std_msgs/Float32 cProb
std_msgs/Float32[] lPoly
std_msgs/Float32 lProb
std_msgs/Float32[] rPoly
std_msgs/Float32 rProb

std_msgs/Float32 angleSteers
std_msgs/Float32 rateSteers
std_msgs/Bool mpcSolutionValid
std_msgs/Bool paramsValid
std_msgs/Float32 angleOffset
std_msgs/Bool sensorValid
std_msgs/Bool commIssue
std_msgs/Bool posenetValid
std_msgs/UInt64 desire
std_msgs/UInt64 laneChangeState
std_msgs/UInt64 laneChangeDirection



# struct PathPlan {
#   laneWidth @0 :Float32;

#   dPoly @1 :List(Float32);
#   cPoly @2 :List(Float32);
#   cProb @3 :Float32;
#   lPoly @4 :List(Float32);
#   lProb @5 :Float32;
#   rPoly @6 :List(Float32);
#   rProb @7 :Float32;

#   angleSteers @8 :Float32; # deg
#   rateSteers @13 :Float32; # deg/s
#   mpcSolutionValid @9 :Bool;
#   paramsValid @10 :Bool;
#   modelValidDEPRECATED @12 :Bool;
#   angleOffset @11 :Float32;
#   sensorValid @14 :Bool;
#   commIssue @15 :Bool;
#   posenetValid @16 :Bool;
#   desire @17 :Desire;
#   laneChangeState @18 :LaneChangeState;
#   laneChangeDirection @19 :LaneChangeDirection;

#   enum Desire {
#     none @0;
#     turnLeft @1;
#     turnRight @2;
#     laneChangeLeft @3;
#     laneChangeRight @4;
#     keepLeft @5;
#     keepRight @6;
#   }

#   enum LaneChangeState {
#     off @0;
#     preLaneChange @1;
#     laneChangeStarting @2;
#     laneChangeFinishing @3;
#   }

#   enum LaneChangeDirection {
#     none @0;
#     left @1;
#     right @2;
#   }
# }
