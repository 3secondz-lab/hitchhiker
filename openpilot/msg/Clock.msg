# Clock Message

std_msgs/Int64 timeNanos
std_msgs/Int32 hardwareClockDiscontinuityCount

std_msgs/Bool hasTimeUncertaintyNanos
std_msgs/Float64 timeUncertaintyNanos

std_msgs/Bool hasLeapSecond
std_msgs/Int32 leapSecond

std_msgs/Bool hasFullBiasNanos
std_msgs/Int64 fullBiasNanos

std_msgs/Bool hasBiasNanos
std_msgs/Float64 biasNanos

std_msgs/Bool hasBiasUncertaintyNanos
std_msgs/Float64 biasUncertaintyNanos

std_msgs/Bool hasDriftNanosPerSecond
std_msgs/Float64 driftNanosPerSecond

std_msgs/Bool hasDriftUncertaintyNanosPerSecond
std_msgs/Float64 driftUncertaintyNanosPerSecond



#    struct Clock {
#      timeNanos @0 :Int64;
#      hardwareClockDiscontinuityCount @1 :Int32;
#
#      hasTimeUncertaintyNanos @2 :Bool;
#      timeUncertaintyNanos @3 :Float64;
#
#      hasLeapSecond @4 :Bool;
#      leapSecond @5 :Int32;
#
#      hasFullBiasNanos @6 :Bool;
#      fullBiasNanos @7 :Int64;
#
#      hasBiasNanos @8 :Bool;
#      biasNanos @9 :Float64;
#
#      hasBiasUncertaintyNanos @10 :Bool;
#      biasUncertaintyNanos @11 :Float64;
#
#      hasDriftNanosPerSecond @12 :Bool;
#      driftNanosPerSecond @13 :Float64;
#
#      hasDriftUncertaintyNanosPerSecond @14 :Bool;
#      driftUncertaintyNanosPerSecond @15 :Float64;
#    }
