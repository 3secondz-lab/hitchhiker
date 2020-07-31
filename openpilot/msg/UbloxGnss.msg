# UbloxGnss Message

std_msgs/Header header

openpilot/MeasurementReportUbloxGnss measurementReport
openpilot/Ephemeris ephemeris
openpilot/IonoData ionoData
openpilot/HwStatus hwStatus



# struct UbloxGnss {
#   union {
#     measurementReport @0 :MeasurementReport;
#     ephemeris @1 :Ephemeris;
#     ionoData @2 :IonoData;
#     hwStatus @3 :HwStatus;
#   }

#   struct MeasurementReport {
#     #received time of week in gps time in seconds and gps week
#     rcvTow @0 :Float64;
#     gpsWeek @1 :UInt16;
#     # leap seconds in seconds
#     leapSeconds @2 :UInt16;
#     # receiver status
#     receiverStatus @3 :ReceiverStatus;
#     # num of measurements to follow
#     numMeas @4 :UInt8;
#     measurements @5 :List(Measurement);

#     struct ReceiverStatus {
#       # leap seconds have been determined
#       leapSecValid @0 :Bool;
#       # Clock reset applied
#       clkReset @1 :Bool;
#     }

#     struct Measurement {
#       svId @0 :UInt8;
#       trackingStatus @1 :TrackingStatus;
#       # pseudorange in meters
#       pseudorange @2 :Float64;
#       # carrier phase measurement in cycles
#       carrierCycles @3 :Float64;
#       # doppler measurement in Hz
#       doppler @4 :Float32;
#       # GNSS id, 0 is gps
#       gnssId @5 :UInt8;
#       glonassFrequencyIndex @6 :UInt8;
#       # carrier phase locktime counter in ms
#       locktime @7 :UInt16;
#       # Carrier-to-noise density ratio (signal strength) in dBHz
#       cno @8 :UInt8;
#       # pseudorange standard deviation in meters
#       pseudorangeStdev @9 :Float32;
#       # carrier phase standard deviation in cycles
#       carrierPhaseStdev @10 :Float32;
#       # doppler standard deviation in Hz
#       dopplerStdev @11 :Float32;
#       sigId @12 :UInt8;

#       struct TrackingStatus {
#         # pseudorange valid
#         pseudorangeValid @0 :Bool;
#         # carrier phase valid
#         carrierPhaseValid @1 :Bool;
#         # half cycle valid
#         halfCycleValid @2 :Bool;
#         # half sycle subtracted from phase
#         halfCycleSubtracted @3 :Bool;
#       }
#     }
#   }

#   struct Ephemeris {
#     # This is according to the rinex (2?) format
#     svId @0 :UInt16;
#     year @1 :UInt16;
#     month @2 :UInt16;
#     day @3 :UInt16;
#     hour @4 :UInt16;
#     minute @5 :UInt16;
#     second @6 :Float32;
#     af0 @7 :Float64;
#     af1 @8 :Float64;
#     af2 @9 :Float64;

#     iode @10 :Float64;
#     crs @11 :Float64;
#     deltaN @12 :Float64;
#     m0 @13 :Float64;

#     cuc @14 :Float64;
#     ecc @15 :Float64;
#     cus @16 :Float64;
#     a @17 :Float64; # note that this is not the root!!

#     toe @18 :Float64;
#     cic @19 :Float64;
#     omega0 @20 :Float64;
#     cis @21 :Float64;

#     i0 @22 :Float64;
#     crc @23 :Float64;
#     omega @24 :Float64;
#     omegaDot @25 :Float64;

#     iDot @26 :Float64;
#     codesL2 @27 :Float64;
#     gpsWeek @28 :Float64;
#     l2 @29 :Float64;

#     svAcc @30 :Float64;
#     svHealth @31 :Float64;
#     tgd @32 :Float64;
#     iodc @33 :Float64;

#     transmissionTime @34 :Float64;
#     fitInterval @35 :Float64;

#     toc @36 :Float64;

#     ionoCoeffsValid @37 :Bool;
#     ionoAlpha @38 :List(Float64);
#     ionoBeta @39 :List(Float64);

#   }

#   struct IonoData {
#     svHealth @0 :UInt32;
#     tow  @1 :Float64;
#     gpsWeek @2 :Float64;

#     ionoAlpha @3 :List(Float64);
#     ionoBeta @4 :List(Float64);

#     healthValid @5 :Bool;
#     ionoCoeffsValid @6 :Bool;
#   }

#   struct HwStatus {
#     noisePerMS @0 :UInt16;
#     agcCnt @1 :UInt16;
#     aStatus @2 :AntennaSupervisorState;
#     aPower @3 :AntennaPowerStatus;
#     jamInd @4 :UInt8;

#     enum AntennaSupervisorState {
#       init @0;
#       dontknow @1;
#       ok @2;
#       short @3;
#       open @4;
#     }

#     enum AntennaPowerStatus {
#       off @0;
#       on @1;
#       dontknow @2;
#     }
#   }
# }
