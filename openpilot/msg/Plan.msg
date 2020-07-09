# Plan Message

std_msgs/Header header

std_msgs/UInt64 mdMonoTime
std_msgs/UInt64 radarStateMonoTime
std_msgs/Bool commIssue

# longitudinal
std_msgs/Float32 vCruise
std_msgs/Float32 aCruise
std_msgs/Float32 vTarget
std_msgs/Float32 vTargetFuture
std_msgs/Float32 vMax
std_msgs/Float32 aTarget

std_msgs/Float32 vStart
std_msgs/Float32 aStart

std_msgs/Float32 jerkFactor
std_msgs/Bool hasLead
std_msgs/Bool fcw
std_msgs/UInt64 longitudinalPlanSource

# gps trajectory in car frame
openpilot/GpsTrajectory gpsTrajectory

std_msgs/Bool gpsPlannerActive

# maps
std_msgs/Float32 vCurvature
std_msgs/Bool decelForTurn
std_msgs/Bool mapValid
std_msgs/Bool radarValid
std_msgs/Bool radarCanError

std_msgs/Float32 processingDelay



# struct Plan {
#   mdMonoTime @9 :UInt64;
#   radarStateMonoTime @10 :UInt64;
#   commIssue @31 :Bool;

#   eventsDEPRECATED @13 :List(Car.CarEvent);

#   # lateral, 3rd order polynomial
#   lateralValidDEPRECATED @0 :Bool;
#   dPolyDEPRECATED @1 :List(Float32);
#   laneWidthDEPRECATED @11 :Float32;

#   # longitudinal
#   longitudinalValidDEPRECATED @2 :Bool;
#   vCruise @16 :Float32;
#   aCruise @17 :Float32;
#   vTarget @3 :Float32;
#   vTargetFuture @14 :Float32;
#   vMax @20 :Float32;
#   aTargetMinDEPRECATED @4 :Float32;
#   aTargetMaxDEPRECATED @5 :Float32;
#   aTarget @18 :Float32;

#   vStart @26 :Float32;
#   aStart @27 :Float32;

#   jerkFactor @6 :Float32;
#   hasLead @7 :Bool;
#   hasLeftLaneDEPRECATED @23 :Bool;
#   hasRightLaneDEPRECATED @24 :Bool;
#   fcw @8 :Bool;
#   longitudinalPlanSource @15 :LongitudinalPlanSource;

#   # gps trajectory in car frame
#   gpsTrajectory @12 :GpsTrajectory;

#   gpsPlannerActive @19 :Bool;

#   # maps
#   vCurvature @21 :Float32;
#   decelForTurn @22 :Bool;
#   mapValid @25 :Bool;
#   radarValid @28 :Bool;
#   radarCanError @30 :Bool;

#   processingDelay @29 :Float32;


#   struct GpsTrajectory {
#     x @0 :List(Float32);
#     y @1 :List(Float32);
#   }

#   enum LongitudinalPlanSource {
#     cruise @0;
#     mpc1 @1;
#     mpc2 @2;
#     mpc3 @3;
#     model @4;
#   }
# }
