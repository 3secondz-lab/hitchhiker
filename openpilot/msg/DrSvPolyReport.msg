# DrSvPolyReport Message

std_msgs/UInt16 svId
std_msgs/Int8 frequencyIndex

std_msgs/Bool hasPosition
std_msgs/Bool hasIono
std_msgs/Bool hasTropo
std_msgs/Bool hasElevation
std_msgs/Bool polyFromXtra
std_msgs/Bool hasSbasIono

std_msgs/UInt16 iode
std_msgs/Float64 t0
std_msgs/Float64[] xyz0
std_msgs/Float64[] xyzN
std_msgs/Float32[] other

std_msgs/Float32 positionUncertainty
std_msgs/Float32 ionoDelay
std_msgs/Float32 ionoDot
std_msgs/Float32 sbasIonoDelay
std_msgs/Float32 sbasIonoDot
std_msgs/Float32 tropoDelay
std_msgs/Float32 elevation
std_msgs/Float32 elevationDot
std_msgs/Float32 elevationUncertainty

std_msgs/Float64[] velocityCoeff



#  struct DrSvPolyReport {
#    svId @0 :UInt16;
#    frequencyIndex @1 :Int8;
#
#    hasPosition @2 :Bool;
#    hasIono @3 :Bool;
#    hasTropo @4 :Bool;
#    hasElevation @5 :Bool;
#    polyFromXtra @6 :Bool;
#    hasSbasIono @7 :Bool;
#
#    iode @8 :UInt16;
#    t0 @9 :Float64;
#    xyz0 @10 :List(Float64);
#    xyzN @11 :List(Float64);
#    other @12 :List(Float32);
#
#    positionUncertainty @13 :Float32;
#    ionoDelay @14 :Float32;
#    ionoDot @15 :Float32;
#    sbasIonoDelay @16 :Float32;
#    sbasIonoDot @17 :Float32;
#    tropoDelay @18 :Float32;
#    elevation @19 :Float32;
#    elevationDot @20 :Float32;
#    elevationUncertainty @21 :Float32;
#
#    velocityCoeff @22 :List(Float64);
#
#  }
