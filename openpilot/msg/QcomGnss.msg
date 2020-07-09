# QcomGnss Message

std_msgs/Header header

std_msgs/UInt64 logTs

openpilot/MeasurementReportQcomGnss measurementReport
openpilot/ClockReport clockReport
openpilot/DrMeasurementReport drMeasurementReport
openpilot/DrSvPolyReport drSvPoly
std_msgs/Byte[] rawLog



# struct QcomGnss {
#   logTs @0 :UInt64;
#   union {
#     measurementReport @1 :MeasurementReport;
#     clockReport @2 :ClockReport;
#     drMeasurementReport @3 :DrMeasurementReport;
#     drSvPoly @4 :DrSvPolyReport;
#     rawLog @5 :Data;
#   }

#   enum MeasurementSource @0xd71a12b6faada7ee {
#     gps @0;
#     glonass @1;
#     beidou @2;
#   }

#   enum SVObservationState @0xe81e829a0d6c83e9 {
#     idle @0;
#     search @1;
#     searchVerify @2;
#     bitEdge @3;
#     trackVerify @4;
#     track @5;
#     restart @6;
#     dpo @7;
#     glo10msBe @8;
#     glo10msAt @9;
#   }

#   struct MeasurementStatus @0xe501010e1bcae83b {
#     subMillisecondIsValid @0 :Bool;
#     subBitTimeIsKnown @1 :Bool;
#     satelliteTimeIsKnown @2 :Bool;
#     bitEdgeConfirmedFromSignal @3 :Bool;
#     measuredVelocity @4 :Bool;
#     fineOrCoarseVelocity @5 :Bool;
#     lockPointValid @6 :Bool;
#     lockPointPositive @7 :Bool;
#     lastUpdateFromDifference @8 :Bool;
#     lastUpdateFromVelocityDifference @9 :Bool;
#     strongIndicationOfCrossCorelation @10 :Bool;
#     tentativeMeasurement @11 :Bool;
#     measurementNotUsable @12 :Bool;
#     sirCheckIsNeeded @13 :Bool;
#     probationMode @14 :Bool;

#     glonassMeanderBitEdgeValid @15 :Bool;
#     glonassTimeMarkValid @16 :Bool;

#     gpsRoundRobinRxDiversity @17 :Bool;
#     gpsRxDiversity @18 :Bool;
#     gpsLowBandwidthRxDiversityCombined @19 :Bool;
#     gpsHighBandwidthNu4 @20 :Bool;
#     gpsHighBandwidthNu8 @21 :Bool;
#     gpsHighBandwidthUniform @22 :Bool;
#     multipathIndicator @23 :Bool;

#     imdJammingIndicator @24 :Bool;
#     lteB13TxJammingIndicator @25 :Bool;
#     freshMeasurementIndicator @26 :Bool;

#     multipathEstimateIsValid @27 :Bool;
#     directionIsValid @28 :Bool;
#   }

#   struct MeasurementReport {
#     source @0 :MeasurementSource;

#     fCount @1 :UInt32;

#     gpsWeek @2 :UInt16;
#     glonassCycleNumber @3 :UInt8;
#     glonassNumberOfDays @4 :UInt16;

#     milliseconds @5 :UInt32;
#     timeBias @6 :Float32;
#     clockTimeUncertainty @7 :Float32;
#     clockFrequencyBias @8 :Float32;
#     clockFrequencyUncertainty @9 :Float32;

#     sv @10 :List(SV);

#     struct SV {
#       svId @0 :UInt8;
#       observationState @2 :SVObservationState;
#       observations @3 :UInt8;
#       goodObservations @4 :UInt8;
#       gpsParityErrorCount @5 :UInt16;
#       glonassFrequencyIndex @1 :Int8;
#       glonassHemmingErrorCount @6 :UInt8;
#       filterStages @7 :UInt8;
#       carrierNoise @8 :UInt16;
#       latency @9 :Int16;
#       predetectInterval @10 :UInt8;
#       postdetections @11 :UInt16;

#       unfilteredMeasurementIntegral @12 :UInt32;
#       unfilteredMeasurementFraction @13 :Float32;
#       unfilteredTimeUncertainty @14 :Float32;
#       unfilteredSpeed @15 :Float32;
#       unfilteredSpeedUncertainty @16 :Float32;
#       measurementStatus @17 :MeasurementStatus;
#       multipathEstimate @18 :UInt32;
#       azimuth @19 :Float32;
#       elevation @20 :Float32;
#       carrierPhaseCyclesIntegral @21 :Int32;
#       carrierPhaseCyclesFraction @22 :UInt16;
#       fineSpeed @23 :Float32;
#       fineSpeedUncertainty @24 :Float32;
#       cycleSlipCount @25 :UInt8;
#     }

#   }

#   struct ClockReport {
#     hasFCount @0 :Bool;
#     fCount @1 :UInt32;

#     hasGpsWeek @2 :Bool;
#     gpsWeek @3 :UInt16;
#     hasGpsMilliseconds @4 :Bool;
#     gpsMilliseconds @5 :UInt32;
#     gpsTimeBias @6 :Float32;
#     gpsClockTimeUncertainty @7 :Float32;
#     gpsClockSource @8 :UInt8;

#     hasGlonassYear @9 :Bool;
#     glonassYear @10 :UInt8;
#     hasGlonassDay @11 :Bool;
#     glonassDay @12 :UInt16;
#     hasGlonassMilliseconds @13 :Bool;
#     glonassMilliseconds @14 :UInt32;
#     glonassTimeBias @15 :Float32;
#     glonassClockTimeUncertainty @16 :Float32;
#     glonassClockSource @17 :UInt8;

#     bdsWeek @18 :UInt16;
#     bdsMilliseconds @19 :UInt32;
#     bdsTimeBias @20 :Float32;
#     bdsClockTimeUncertainty @21 :Float32;
#     bdsClockSource @22 :UInt8;

#     galWeek @23 :UInt16;
#     galMilliseconds @24 :UInt32;
#     galTimeBias @25 :Float32;
#     galClockTimeUncertainty @26 :Float32;
#     galClockSource @27 :UInt8;

#     clockFrequencyBias @28 :Float32;
#     clockFrequencyUncertainty @29 :Float32;
#     frequencySource @30 :UInt8;
#     gpsLeapSeconds @31 :UInt8;
#     gpsLeapSecondsUncertainty @32 :UInt8;
#     gpsLeapSecondsSource @33 :UInt8;

#     gpsToGlonassTimeBiasMilliseconds @34 :Float32;
#     gpsToGlonassTimeBiasMillisecondsUncertainty @35 :Float32;
#     gpsToBdsTimeBiasMilliseconds @36 :Float32;
#     gpsToBdsTimeBiasMillisecondsUncertainty @37 :Float32;
#     bdsToGloTimeBiasMilliseconds @38 :Float32;
#     bdsToGloTimeBiasMillisecondsUncertainty @39 :Float32;
#     gpsToGalTimeBiasMilliseconds @40 :Float32;
#     gpsToGalTimeBiasMillisecondsUncertainty @41 :Float32;
#     galToGloTimeBiasMilliseconds @42 :Float32;
#     galToGloTimeBiasMillisecondsUncertainty @43 :Float32;
#     galToBdsTimeBiasMilliseconds @44 :Float32;
#     galToBdsTimeBiasMillisecondsUncertainty @45 :Float32;

#     hasRtcTime @46 :Bool;
#     systemRtcTime @47 :UInt32;
#     fCountOffset @48 :UInt32;
#     lpmRtcCount @49 :UInt32;
#     clockResets @50 :UInt32;
#   }

#   struct DrMeasurementReport {

#     reason @0 :UInt8;
#     seqNum @1 :UInt8;
#     seqMax @2 :UInt8;
#     rfLoss @3 :UInt16;

#     systemRtcValid @4 :Bool;
#     fCount @5 :UInt32;
#     clockResets @6 :UInt32;
#     systemRtcTime @7 :UInt64;

#     gpsLeapSeconds @8 :UInt8;
#     gpsLeapSecondsUncertainty @9 :UInt8;
#     gpsToGlonassTimeBiasMilliseconds @10 :Float32;
#     gpsToGlonassTimeBiasMillisecondsUncertainty @11 :Float32;

#     gpsWeek @12 :UInt16;
#     gpsMilliseconds @13 :UInt32;
#     gpsTimeBiasMs @14 :UInt32;
#     gpsClockTimeUncertaintyMs @15 :UInt32;
#     gpsClockSource @16 :UInt8;

#     glonassClockSource @17 :UInt8;
#     glonassYear @18 :UInt8;
#     glonassDay @19 :UInt16;
#     glonassMilliseconds @20 :UInt32;
#     glonassTimeBias @21 :Float32;
#     glonassClockTimeUncertainty @22 :Float32;

#     clockFrequencyBias @23 :Float32;
#     clockFrequencyUncertainty @24 :Float32;
#     frequencySource @25 :UInt8;

#     source @26 :MeasurementSource;

#     sv @27 :List(SV);

#     struct SV {
#       svId @0 :UInt8;
#       glonassFrequencyIndex @1 :Int8;
#       observationState @2 :SVObservationState;
#       observations @3 :UInt8;
#       goodObservations @4 :UInt8;
#       filterStages @5 :UInt8;
#       predetectInterval @6 :UInt8;
#       cycleSlipCount @7 :UInt8;
#       postdetections @8 :UInt16;

#       measurementStatus @9 :MeasurementStatus;

#       carrierNoise @10 :UInt16;
#       rfLoss @11 :UInt16;
#       latency @12 :Int16;

#       filteredMeasurementFraction @13 :Float32;
#       filteredMeasurementIntegral @14 :UInt32;
#       filteredTimeUncertainty @15 :Float32;
#       filteredSpeed @16 :Float32;
#       filteredSpeedUncertainty @17 :Float32;

#       unfilteredMeasurementFraction @18 :Float32;
#       unfilteredMeasurementIntegral @19 :UInt32;
#       unfilteredTimeUncertainty @20 :Float32;
#       unfilteredSpeed @21 :Float32;
#       unfilteredSpeedUncertainty @22 :Float32;

#       multipathEstimate @23 :UInt32;
#       azimuth @24 :Float32;
#       elevation @25 :Float32;
#       dopplerAcceleration @26 :Float32;
#       fineSpeed @27 :Float32;
#       fineSpeedUncertainty @28 :Float32;

#       carrierPhase @29 :Float64;
#       fCount @30 :UInt32;

#       parityErrorCount @31 :UInt16;
#       goodParity @32 :Bool;
#     }
#   }

#   struct DrSvPolyReport {
#     svId @0 :UInt16;
#     frequencyIndex @1 :Int8;

#     hasPosition @2 :Bool;
#     hasIono @3 :Bool;
#     hasTropo @4 :Bool;
#     hasElevation @5 :Bool;
#     polyFromXtra @6 :Bool;
#     hasSbasIono @7 :Bool;

#     iode @8 :UInt16;
#     t0 @9 :Float64;
#     xyz0 @10 :List(Float64);
#     xyzN @11 :List(Float64);
#     other @12 :List(Float32);

#     positionUncertainty @13 :Float32;
#     ionoDelay @14 :Float32;
#     ionoDot @15 :Float32;
#     sbasIonoDelay @16 :Float32;
#     sbasIonoDot @17 :Float32;
#     tropoDelay @18 :Float32;
#     elevation @19 :Float32;
#     elevationDot @20 :Float32;
#     elevationUncertainty @21 :Float32;

#     velocityCoeff @22 :List(Float64);

#   }
# }
