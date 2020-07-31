# MeasurementStatus Message

std_msgs/Bool subMillisecondIsValid
std_msgs/Bool subBitTimeIsKnown
std_msgs/Bool satelliteTimeIsKnown
std_msgs/Bool bitEdgeConfirmedFromSignal
std_msgs/Bool measuredVelocity
std_msgs/Bool fineOrCoarseVelocity
std_msgs/Bool lockPointValid
std_msgs/Bool lockPointPositive
std_msgs/Bool lastUpdateFromDifference
std_msgs/Bool lastUpdateFromVelocityDifference
std_msgs/Bool strongIndicationOfCrossCorelation
std_msgs/Bool tentativeMeasurement
std_msgs/Bool measurementNotUsable
std_msgs/Bool sirCheckIsNeeded
std_msgs/Bool probationMode

std_msgs/Bool glonassMeanderBitEdgeValid
std_msgs/Bool glonassTimeMarkValid

std_msgs/Bool gpsRoundRobinRxDiversity
std_msgs/Bool gpsRxDiversity
std_msgs/Bool gpsLowBandwidthRxDiversityCombined
std_msgs/Bool gpsHighBandwidthNu4
std_msgs/Bool gpsHighBandwidthNu8
std_msgs/Bool gpsHighBandwidthUniform
std_msgs/Bool multipathIndicator

std_msgs/Bool imdJammingIndicator
std_msgs/Bool lteB13TxJammingIndicator
std_msgs/Bool freshMeasurementIndicator

std_msgs/Bool multipathEstimateIsValid
std_msgs/Bool directionIsValid



#  struct MeasurementStatus @0xe501010e1bcae83b {
#    subMillisecondIsValid @0 :Bool;
#    subBitTimeIsKnown @1 :Bool;
#    satelliteTimeIsKnown @2 :Bool;
#    bitEdgeConfirmedFromSignal @3 :Bool;
#    measuredVelocity @4 :Bool;
#    fineOrCoarseVelocity @5 :Bool;
#    lockPointValid @6 :Bool;
#    lockPointPositive @7 :Bool;
#    lastUpdateFromDifference @8 :Bool;
#    lastUpdateFromVelocityDifference @9 :Bool;
#    strongIndicationOfCrossCorelation @10 :Bool;
#    tentativeMeasurement @11 :Bool;
#    measurementNotUsable @12 :Bool;
#    sirCheckIsNeeded @13 :Bool;
#    probationMode @14 :Bool;
#
#    glonassMeanderBitEdgeValid @15 :Bool;
#    glonassTimeMarkValid @16 :Bool;
#
#    gpsRoundRobinRxDiversity @17 :Bool;
#    gpsRxDiversity @18 :Bool;
#    gpsLowBandwidthRxDiversityCombined @19 :Bool;
#    gpsHighBandwidthNu4 @20 :Bool;
#    gpsHighBandwidthNu8 @21 :Bool;
#    gpsHighBandwidthUniform @22 :Bool;
#    multipathIndicator @23 :Bool;
#
#    imdJammingIndicator @24 :Bool;
#    lteB13TxJammingIndicator @25 :Bool;
#    freshMeasurementIndicator @26 :Bool;
#
#    multipathEstimateIsValid @27 :Bool;
#    directionIsValid @28 :Bool;
#  }
