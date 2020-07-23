# Measurements Message

openpilot/Clock clock
openpilot/MeasurementAndroidGnss[] measurements



#  struct Measurements {
#    clock @0 :Clock;
#    measurements @1 :List(Measurement);
#
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
#
#    struct Measurement {
#      svId @0 :Int32;
#      constellation @1 :Constellation;
#
#      timeOffsetNanos @2 :Float64;
#      state @3 :Int32;
#      receivedSvTimeNanos @4 :Int64;
#      receivedSvTimeUncertaintyNanos @5 :Int64;
#      cn0DbHz @6 :Float64;
#      pseudorangeRateMetersPerSecond @7 :Float64;
#      pseudorangeRateUncertaintyMetersPerSecond @8 :Float64;
#      accumulatedDeltaRangeState @9 :Int32;
#      accumulatedDeltaRangeMeters @10 :Float64;
#      accumulatedDeltaRangeUncertaintyMeters @11 :Float64;
#
#      hasCarrierFrequencyHz @12 :Bool;
#      carrierFrequencyHz @13 :Float32;
#      hasCarrierCycles @14 :Bool;
#      carrierCycles @15 :Int64;
#      hasCarrierPhase @16 :Bool;
#      carrierPhase @17 :Float64;
#      hasCarrierPhaseUncertainty @18 :Bool;
#      carrierPhaseUncertainty @19 :Float64;
#      hasSnrInDb @20 :Bool;
#      snrInDb @21 :Float64;
#
#      multipathIndicator @22 :MultipathIndicator;
#
#      enum Constellation {
#        unknown @0;
#        gps @1;
#        sbas @2;
#        glonass @3;
#        qzss @4;
#        beidou @5;
#        galileo @6;
#      }
#
#      enum State {
#        unknown @0;
#        codeLock @1;
#        bitSync @2;
#        subframeSync @3;
#        towDecoded @4;
#        msecAmbiguous @5;
#        symbolSync @6;
#        gloStringSync @7;
#        gloTodDecoded @8;
#        bdsD2BitSync @9;
#        bdsD2SubframeSync @10;
#        galE1bcCodeLock @11;
#        galE1c2ndCodeLock @12;
#        galE1bPageSync @13;
#        sbasSync @14;
#      }
#
#      enum MultipathIndicator {
#        unknown @0;
#        detected @1;
#        notDetected @2;
#      }
#    }
#  }
#
#  struct NavigationMessage {
#    type @0 :Int32;
#    svId @1 :Int32;
#    messageId @2 :Int32;
#    submessageId @3 :Int32;
#    data @4 :Data;
#    status @5 :Status;
#
#    enum Status {
#      unknown @0;
#      parityPassed @1;
#      parityRebuilt @2;
#    }
#  }
