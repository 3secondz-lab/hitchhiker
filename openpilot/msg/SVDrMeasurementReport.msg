# SVDrMeasurementReport Message

std_msgs/UInt8 svId
std_msgs/Int8 glonassFrequencyIndex
std_msgs/String observationState
std_msgs/UInt8 observations
std_msgs/UInt8 goodObservations
std_msgs/UInt8 filterStages
std_msgs/UInt8 predetectInterval
std_msgs/UInt8 cycleSlipCount
std_msgs/UInt16 postdetections

openpilot/MeasurementStatus measurementStatus

std_msgs/UInt16 carrierNoise
std_msgs/UInt16 rfLoss
std_msgs/Int16 latency

std_msgs/Float32 filteredMeasurementFraction
std_msgs/UInt32 filteredMeasurementIntegral
std_msgs/Float32 filteredTimeUncertainty
std_msgs/Float32 filteredSpeed
std_msgs/Float32 filteredSpeedUncertainty

std_msgs/Float32 unfilteredMeasurementFraction
std_msgs/UInt32 unfilteredMeasurementIntegral
std_msgs/Float32 unfilteredTimeUncertainty
std_msgs/Float32 unfilteredSpeed
std_msgs/Float32 unfilteredSpeedUncertainty

std_msgs/UInt32 multipathEstimate
std_msgs/Float32 azimuth
std_msgs/Float32 elevation
std_msgs/Float32 dopplerAcceleration
std_msgs/Float32 fineSpeed
std_msgs/Float32 fineSpeedUncertainty

std_msgs/Float64 carrierPhase
std_msgs/UInt32 fCount

std_msgs/UInt16 parityErrorCount
std_msgs/Bool goodParity



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
