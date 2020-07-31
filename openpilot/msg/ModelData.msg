# ModelData Message

std_msgs/Header header

std_msgs/UInt32 frameId
std_msgs/UInt32 frameAge
std_msgs/Float32 frameDropPerc
std_msgs/UInt64 timestampEof

openpilot/PathData path
openpilot/PathData leftLane
openpilot/PathData rightLane
openpilot/LeadDataModel lead
std_msgs/Float32[] freePath

openpilot/ModelSettings settings
openpilot/LeadDataModel leadFuture
std_msgs/Float32[] speed
openpilot/MetaData meta
openpilot/LongitudinalData longitudinal



#struct ModelData {
#  frameId @0 :UInt32;
#  frameAge @12 :UInt32;
#  frameDropPerc @13 :Float32;
#  timestampEof @9 :UInt64;
#
#  path @1 :PathData;
#  leftLane @2 :PathData;
#  rightLane @3 :PathData;
#  lead @4 :LeadData;
#  freePath @6 :List(Float32);
#
#  settings @5 :ModelSettings;
#  leadFuture @7 :LeadData;
#  speed @8 :List(Float32);
#  meta @10 :MetaData;
#  longitudinal @11 :LongitudinalData;
#
#  struct PathData {
#    points @0 :List(Float32);
#    prob @1 :Float32;
#    std @2 :Float32;
#    stds @3 :List(Float32);
#    poly @4 :List(Float32);
#    validLen @5 :Float32;
#  }
#
#  struct LeadData {
#    dist @0 :Float32;
#    prob @1 :Float32;
#    std @2 :Float32;
#    relVel @3 :Float32;
#    relVelStd @4 :Float32;
#    relY @5 :Float32;
#    relYStd @6 :Float32;
#    relA @7 :Float32;
#    relAStd @8 :Float32;
#  }
#
#  struct ModelSettings {
#    bigBoxX @0 :UInt16;
#    bigBoxY @1 :UInt16;
#    bigBoxWidth @2 :UInt16;
#    bigBoxHeight @3 :UInt16;
#    boxProjection @4 :List(Float32);
#    yuvCorrection @5 :List(Float32);
#    inputTransform @6 :List(Float32);
#  }
#
#  struct MetaData {
#    engagedProb @0 :Float32;
#    desirePrediction @1 :List(Float32);
#    brakeDisengageProb @2 :Float32;
#    gasDisengageProb @3 :Float32;
#    steerOverrideProb @4 :Float32;
#    desireState @5 :List(Float32);
#  }
#
#  struct LongitudinalData {
#    distances @2 :List(Float32);
#    speeds @0 :List(Float32);
#    accelerations @1 :List(Float32);
#  }
#}
