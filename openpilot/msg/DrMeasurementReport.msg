# DrMeasurementReport Message

std_msgs/UInt8 reason
std_msgs/UInt8 seqNum
std_msgs/UInt8 seqMax
std_msgs/UInt16 rfLoss

std_msgs/Bool systemRtcValid
std_msgs/UInt32 fCount
std_msgs/UInt32 clockResets
std_msgs/UInt64 systemRtcTime

std_msgs/UInt8 gpsLeapSeconds
std_msgs/UInt8 gpsLeapSecondsUncertainty
std_msgs/Float32 gpsToGlonassTimeBiasMilliseconds
std_msgs/Float32 gpsToGlonassTimeBiasMillisecondsUncertainty

std_msgs/UInt16 gpsWeek
std_msgs/UInt32 gpsMilliseconds
std_msgs/UInt32 gpsTimeBiasMs
std_msgs/UInt32 gpsClockTimeUncertaintyMs
std_msgs/UInt8 gpsClockSource

std_msgs/UInt8 glonassClockSource
std_msgs/UInt8 glonassYear
std_msgs/UInt16 glonassDay
std_msgs/UInt32 glonassMilliseconds
std_msgs/Float32 glonassTimeBias
std_msgs/Float32 glonassClockTimeUncertainty

std_msgs/Float32 clockFrequencyBias
std_msgs/Float32 clockFrequencyUncertainty
std_msgs/UInt8 frequencySource

std_msgs/UInt64 source

openpilot/SVDrMeasurementReport[] sv



#struct DrMeasurementReport {
#
#    reason @0 :UInt8;
#    seqNum @1 :UInt8;
#    seqMax @2 :UInt8;
#    rfLoss @3 :UInt16;
#
#    systemRtcValid @4 :Bool;
#    fCount @5 :UInt32;
#    clockResets @6 :UInt32;
#    systemRtcTime @7 :UInt64;
#
#    gpsLeapSeconds @8 :UInt8;
#    gpsLeapSecondsUncertainty @9 :UInt8;
#    gpsToGlonassTimeBiasMilliseconds @10 :Float32;
#    gpsToGlonassTimeBiasMillisecondsUncertainty @11 :Float32;
#
#    gpsWeek @12 :UInt16;
#    gpsMilliseconds @13 :UInt32;
#    gpsTimeBiasMs @14 :UInt32;
#    gpsClockTimeUncertaintyMs @15 :UInt32;
#    gpsClockSource @16 :UInt8;
#
#    glonassClockSource @17 :UInt8;
#    glonassYear @18 :UInt8;
#    glonassDay @19 :UInt16;
#    glonassMilliseconds @20 :UInt32;
#    glonassTimeBias @21 :Float32;
#    glonassClockTimeUncertainty @22 :Float32;
#
#    clockFrequencyBias @23 :Float32;
#    clockFrequencyUncertainty @24 :Float32;
#    frequencySource @25 :UInt8;
#
#    source @26 :MeasurementSource;
#
#    sv @27 :List(SV);
#
#    struct SV {
#      svId @0 :UInt8;
#      glonassFrequencyIndex @1 :Int8;
#      observationState @2 :SVObservationState;
#      observations @3 :UInt8;
#      goodObservations @4 :UInt8;
#      filterStages @5 :UInt8;
#      predetectInterval @6 :UInt8;
#      cycleSlipCount @7 :UInt8;
#      postdetections @8 :UInt16;
#
#      measurementStatus @9 :MeasurementStatus;
#
#      carrierNoise @10 :UInt16;
#      rfLoss @11 :UInt16;
#      latency @12 :Int16;
#
#      filteredMeasurementFraction @13 :Float32;
#      filteredMeasurementIntegral @14 :UInt32;
#      filteredTimeUncertainty @15 :Float32;
#      filteredSpeed @16 :Float32;
#      filteredSpeedUncertainty @17 :Float32;
#
#      unfilteredMeasurementFraction @18 :Float32;
#      unfilteredMeasurementIntegral @19 :UInt32;
#      unfilteredTimeUncertainty @20 :Float32;
#      unfilteredSpeed @21 :Float32;
#      unfilteredSpeedUncertainty @22 :Float32;
#
#      multipathEstimate @23 :UInt32;
#      azimuth @24 :Float32;
#      elevation @25 :Float32;
#      dopplerAcceleration @26 :Float32;
#      fineSpeed @27 :Float32;
#      fineSpeedUncertainty @28 :Float32;
#
#      carrierPhase @29 :Float64;
#      fCount @30 :UInt32;
#
#      parityErrorCount @31 :UInt16;
#      goodParity @32 :Bool;
#    }
#  }
#
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
#}
