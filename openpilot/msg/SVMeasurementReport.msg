# SVMeasurementReport Message

std_msgs/UInt8 svId
std_msgs/String observationState
std_msgs/UInt8 observations
std_msgs/UInt8 goodObservations
std_msgs/UInt16 gpsParityErrorCount
std_msgs/Int8 glonassFrequencyIndex
std_msgs/UInt8 glonassHemmingErrorCount
std_msgs/UInt8 filterStages
std_msgs/UInt16 carrierNoise
std_msgs/Int16 latency
std_msgs/UInt8 predetectInterval
std_msgs/UInt16 postdetections

std_msgs/UInt32 unfilteredMeasurementIntegral
std_msgs/Float32 unfilteredMeasurementFraction
std_msgs/Float32 unfilteredTimeUncertainty
std_msgs/Float32 unfilteredSpeed
std_msgs/Float32 unfilteredSpeedUncertainty
openpilot/MeasurementStatus measurementStatus
std_msgs/UInt32 multipathEstimate
std_msgs/Float32 azimuth
std_msgs/Float32 elevation
std_msgs/Int32 carrierPhaseCyclesIntegral
std_msgs/UInt16 carrierPhaseCyclesFraction
std_msgs/Float32 fineSpeed
std_msgs/Float32 fineSpeedUncertainty
std_msgs/UInt8 cycleSlipCount



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
