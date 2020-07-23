# Ephemeris Message

# This is according to the rinex (2?) format
std_msgs/UInt16 svId
std_msgs/UInt16 year
std_msgs/UInt16 month
std_msgs/UInt16 day
std_msgs/UInt16 hour
std_msgs/UInt16 minute
std_msgs/Float32 second
std_msgs/Float64 af0
std_msgs/Float64 af1
std_msgs/Float64 af2

std_msgs/Float64 iode
std_msgs/Float64 crs
std_msgs/Float64 deltaN
std_msgs/Float64 m0

std_msgs/Float64 cuc
std_msgs/Float64 ecc
std_msgs/Float64 cus
std_msgs/Float64 a

std_msgs/Float64 toe
std_msgs/Float64 cic
std_msgs/Float64 omega0
std_msgs/Float64 cis

std_msgs/Float64 i0
std_msgs/Float64 crc
std_msgs/Float64 omega
std_msgs/Float64 omegaDot

std_msgs/Float64 iDot
std_msgs/Float64 codesL2
std_msgs/Float64 gpsWeek
std_msgs/Float64 l2

std_msgs/Float64 svAcc
std_msgs/Float64 svHealth
std_msgs/Float64 tgd
std_msgs/Float64 iodc

std_msgs/Float64 transmissionTime
std_msgs/Float64 fitInterval

std_msgs/Float64 toc

std_msgs/Bool ionoCoeffsValid
std_msgs/Float64[] ionoAlpha
std_msgs/Float64[] ionoBeta



#struct Ephemeris {
#    # This is according to the rinex (2?) format
#    svId @0 :UInt16;
#    year @1 :UInt16;
#    month @2 :UInt16;
#    day @3 :UInt16;
#    hour @4 :UInt16;
#    minute @5 :UInt16;
#    second @6 :Float32;
#    af0 @7 :Float64;
#    af1 @8 :Float64;
#    af2 @9 :Float64;
#
#    iode @10 :Float64;
#    crs @11 :Float64;
#    deltaN @12 :Float64;
#    m0 @13 :Float64;
#
#    cuc @14 :Float64;
#    ecc @15 :Float64;
#    cus @16 :Float64;
#    a @17 :Float64; # note that this is not the root!!
#
#    toe @18 :Float64;
#    cic @19 :Float64;
#    omega0 @20 :Float64;
#    cis @21 :Float64;
#
#    i0 @22 :Float64;
#    crc @23 :Float64;
#    omega @24 :Float64;
#    omegaDot @25 :Float64;
#
#    iDot @26 :Float64;
#    codesL2 @27 :Float64;
#    gpsWeek @28 :Float64;
#    l2 @29 :Float64;
#
#    svAcc @30 :Float64;
#    svHealth @31 :Float64;
#    tgd @32 :Float64;
#    iodc @33 :Float64;
#
#    transmissionTime @34 :Float64;
#    fitInterval @35 :Float64;
#
#    toc @36 :Float64;
#
#    ionoCoeffsValid @37 :Bool;
#    ionoAlpha @38 :List(Float64);
#    ionoBeta @39 :List(Float64);
#
#  }
