# RadarState Message

std_msgs/Header header

std_msgs/UInt64[] canMonoTimes
std_msgs/UInt64 mdMonoTime
std_msgs/UInt64 controlsStateMonoTime
std_msgs/UInt64[] radarErrors

openpilot/LeadDataRadar leadOne
openpilot/LeadDataRadar leadTwo
std_msgs/Float32 cumLagMs



# struct RadarState @0x9a185389d6fdd05f {
#   canMonoTimes @10 :List(UInt64);
#   mdMonoTime @6 :UInt64;
#   ftMonoTimeDEPRECATED @7 :UInt64;
#   controlsStateMonoTime @11 :UInt64;
#   radarErrors @12 :List(Car.RadarData.Error);

#   # all deprecated
#   warpMatrixDEPRECATED @0 :List(Float32);
#   angleOffsetDEPRECATED @1 :Float32;
#   calStatusDEPRECATED @2 :Int8;
#   calCycleDEPRECATED @8 :Int32;
#   calPercDEPRECATED @9 :Int8;

#   leadOne @3 :LeadData;
#   leadTwo @4 :LeadData;
#   cumLagMs @5 :Float32;

#   struct LeadData {
#     dRel @0 :Float32;
#     yRel @1 :Float32;
#     vRel @2 :Float32;
#     aRel @3 :Float32;
#     vLead @4 :Float32;
#     aLeadDEPRECATED @5 :Float32;
#     dPath @6 :Float32;
#     vLat @7 :Float32;
#     vLeadK @8 :Float32;
#     aLeadK @9 :Float32;
#     fcw @10 :Bool;
#     status @11 :Bool;
#     aLeadTau @12 :Float32;
#     modelProb @13 :Float32;
#     radar @14 :Bool;
#   }
# }
