# SVDrMeasurementReport Message

std_msgs/UInt8 svId
std_msgs/Int8 glonassFrequencyIndex
std_msgs/UInt64 observationState
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