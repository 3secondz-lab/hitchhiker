# ClockReport Message

std_msgs/Bool hasFCount
std_msgs/UInt32 fCount

std_msgs/Bool hasGpsWeek
std_msgs/UInt16 gpsWeek
std_msgs/Bool hasGpsMilliseconds
std_msgs/UInt32 gpsMilliseconds
std_msgs/Float32 gpsTimeBias
std_msgs/Float32 gpsClockTimeUncertainty
std_msgs/UInt8 gpsClockSource

std_msgs/Bool hasGlonassYear
std_msgs/UInt8 glonassYear
std_msgs/Bool hasGlonassDay
std_msgs/UInt16 glonassDay
std_msgs/Bool hasGlonassMilliseconds
std_msgs/UInt32 glonassMilliseconds
std_msgs/Float32 glonassTimeBias
std_msgs/Float32 glonassClockTimeUncertainty
std_msgs/UInt8 glonassClockSource

std_msgs/UInt16 bdsWeek
std_msgs/UInt32 bdsMilliseconds
std_msgs/Float32 bdsTimeBias
std_msgs/Float32 bdsClockTimeUncertainty
std_msgs/UInt8 bdsClockSource

std_msgs/UInt16 galWeek
std_msgs/UInt32 galMilliseconds
std_msgs/Float32 galTimeBias
std_msgs/Float32 galClockTimeUncertainty
std_msgs/UInt8 galClockSource

std_msgs/Float32 clockFrequencyBias
std_msgs/Float32 clockFrequencyUncertainty
std_msgs/UInt8 frequencySource
std_msgs/UInt8 gpsLeapSeconds
std_msgs/UInt8 gpsLeapSecondsUncertainty
std_msgs/UInt8 gpsLeapSecondsSource

std_msgs/Float32 gpsToGlonassTimeBiasMilliseconds
std_msgs/Float32 gpsToGlonassTimeBiasMillisecondsUncertainty
std_msgs/Float32 gpsToBdsTimeBiasMilliseconds
std_msgs/Float32 gpsToBdsTimeBiasMillisecondsUncertainty
std_msgs/Float32 bdsToGloTimeBiasMilliseconds
std_msgs/Float32 bdsToGloTimeBiasMillisecondsUncertainty
std_msgs/Float32 gpsToGalTimeBiasMilliseconds
std_msgs/Float32 gpsToGalTimeBiasMillisecondsUncertainty
std_msgs/Float32 galToGloTimeBiasMilliseconds
std_msgs/Float32 galToGloTimeBiasMillisecondsUncertainty
std_msgs/Float32 galToBdsTimeBiasMilliseconds
std_msgs/Float32 galToBdsTimeBiasMillisecondsUncertainty

std_msgs/Bool hasRtcTime
std_msgs/UInt32 systemRtcTime
std_msgs/UInt32 fCountOffset
std_msgs/UInt32 lpmRtcCount
std_msgs/UInt32 clockResets



#  struct ClockReport {
#    hasFCount @0 :Bool;
#    fCount @1 :UInt32;
#
#    hasGpsWeek @2 :Bool;
#    gpsWeek @3 :UInt16;
#    hasGpsMilliseconds @4 :Bool;
#    gpsMilliseconds @5 :UInt32;
#    gpsTimeBias @6 :Float32;
#    gpsClockTimeUncertainty @7 :Float32;
#    gpsClockSource @8 :UInt8;
#
#    hasGlonassYear @9 :Bool;
#    glonassYear @10 :UInt8;
#    hasGlonassDay @11 :Bool;
#    glonassDay @12 :UInt16;
#    hasGlonassMilliseconds @13 :Bool;
#    glonassMilliseconds @14 :UInt32;
#    glonassTimeBias @15 :Float32;
#    glonassClockTimeUncertainty @16 :Float32;
#    glonassClockSource @17 :UInt8;
#
#    bdsWeek @18 :UInt16;
#    bdsMilliseconds @19 :UInt32;
#    bdsTimeBias @20 :Float32;
#    bdsClockTimeUncertainty @21 :Float32;
#    bdsClockSource @22 :UInt8;
#
#    galWeek @23 :UInt16;
#    galMilliseconds @24 :UInt32;
#    galTimeBias @25 :Float32;
#    galClockTimeUncertainty @26 :Float32;
#    galClockSource @27 :UInt8;
#
#    clockFrequencyBias @28 :Float32;
#    clockFrequencyUncertainty @29 :Float32;
#    frequencySource @30 :UInt8;
#    gpsLeapSeconds @31 :UInt8;
#    gpsLeapSecondsUncertainty @32 :UInt8;
#    gpsLeapSecondsSource @33 :UInt8;
#
#    gpsToGlonassTimeBiasMilliseconds @34 :Float32;
#    gpsToGlonassTimeBiasMillisecondsUncertainty @35 :Float32;
#    gpsToBdsTimeBiasMilliseconds @36 :Float32;
#    gpsToBdsTimeBiasMillisecondsUncertainty @37 :Float32;
#    bdsToGloTimeBiasMilliseconds @38 :Float32;
#    bdsToGloTimeBiasMillisecondsUncertainty @39 :Float32;
#    gpsToGalTimeBiasMilliseconds @40 :Float32;
#    gpsToGalTimeBiasMillisecondsUncertainty @41 :Float32;
#    galToGloTimeBiasMilliseconds @42 :Float32;
#    galToGloTimeBiasMillisecondsUncertainty @43 :Float32;
#    galToBdsTimeBiasMilliseconds @44 :Float32;
#    galToBdsTimeBiasMillisecondsUncertainty @45 :Float32;
#
#    hasRtcTime @46 :Bool;
#    systemRtcTime @47 :UInt32;
#    fCountOffset @48 :UInt32;
#    lpmRtcCount @49 :UInt32;
#    clockResets @50 :UInt32;
#  }
