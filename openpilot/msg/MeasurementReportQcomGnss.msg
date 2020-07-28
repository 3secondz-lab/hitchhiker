# MeasurementReportQcomGnss Message

std_msgs/String source

std_msgs/UInt32 fCount

std_msgs/UInt16 gpsWeek
std_msgs/UInt8 glonassCycleNumber
std_msgs/UInt16 glonassNumberOfDays

std_msgs/UInt32 milliseconds
std_msgs/Float32 timeBias
std_msgs/Float32 clockTimeUncertainty
std_msgs/Float32 clockFrequencyBias
std_msgs/Float32 clockFrequencyUncertainty

openpilot/SVMeasurementReport[] sv



#  struct MeasurementReport {
#    source @0 :MeasurementSource;
#
#    fCount @1 :UInt32;
#
#    gpsWeek @2 :UInt16;
#    glonassCycleNumber @3 :UInt8;
#    glonassNumberOfDays @4 :UInt16;
#
#    milliseconds @5 :UInt32;
#    timeBias @6 :Float32;
#    clockTimeUncertainty @7 :Float32;
#    clockFrequencyBias @8 :Float32;
#    clockFrequencyUncertainty @9 :Float32;
#
#    sv @10 :List(SV);
#
#    struct SV {
#      svId @0 :UInt8;
#      observationState @2 :SVObservationState;
#      observations @3 :UInt8;
#      goodObservations @4 :UInt8;
#      gpsParityErrorCount @5 :UInt16;
#      glonassFrequencyIndex @1 :Int8;
#      glonassHemmingErrorCount @6 :UInt8;
#      filterStages @7 :UInt8;
#      carrierNoise @8 :UInt16;
#      latency @9 :Int16;
#      predetectInterval @10 :UInt8;
#      postdetections @11 :UInt16;
#
#      unfilteredMeasurementIntegral @12 :UInt32;
#      unfilteredMeasurementFraction @13 :Float32;
#      unfilteredTimeUncertainty @14 :Float32;
#      unfilteredSpeed @15 :Float32;
#      unfilteredSpeedUncertainty @16 :Float32;
#      measurementStatus @17 :MeasurementStatus;
#      multipathEstimate @18 :UInt32;
#      azimuth @19 :Float32;
#      elevation @20 :Float32;
#      carrierPhaseCyclesIntegral @21 :Int32;
#      carrierPhaseCyclesFraction @22 :UInt16;
#      fineSpeed @23 :Float32;
#      fineSpeedUncertainty @24 :Float32;
#      cycleSlipCount @25 :UInt8;
#    }
#
#  }
