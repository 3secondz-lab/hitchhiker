# MeasurementReportUbloxGnss Message

#received time of week in gps time in seconds and gps week
std_msgs/Float64 rcvTow
std_msgs/UInt16 gpsWeek
# leap seconds in seconds
std_msgs/UInt16 leapSeconds
# receiver status
openpilot/ReceiverStatus receiverStatus
# num of measurements to follow
std_msgs/UInt8 numMeas
openpilot/MeasurementUbloxGnss[] measurements



#  struct MeasurementReport {
#    #received time of week in gps time in seconds and gps week
#    rcvTow @0 :Float64;
#    gpsWeek @1 :UInt16;
#    # leap seconds in seconds
#    leapSeconds @2 :UInt16;
#    # receiver status
#    receiverStatus @3 :ReceiverStatus;
#    # num of measurements to follow
#    numMeas @4 :UInt8;
#    measurements @5 :List(Measurement);
#
#    struct ReceiverStatus {
#      # leap seconds have been determined
#      leapSecValid @0 :Bool;
#      # Clock reset applied
#      clkReset @1 :Bool;
#    }
#
#    struct Measurement {
#      svId @0 :UInt8;
#      trackingStatus @1 :TrackingStatus;
#      # pseudorange in meters
#      pseudorange @2 :Float64;
#      # carrier phase measurement in cycles
#      carrierCycles @3 :Float64;
#      # doppler measurement in Hz
#      doppler @4 :Float32;
#      # GNSS id, 0 is gps
#      gnssId @5 :UInt8;
#      glonassFrequencyIndex @6 :UInt8;
#      # carrier phase locktime counter in ms
#      locktime @7 :UInt16;
#      # Carrier-to-noise density ratio (signal strength) in dBHz
#      cno @8 :UInt8;
#      # pseudorange standard deviation in meters
#      pseudorangeStdev @9 :Float32;
#      # carrier phase standard deviation in cycles
#      carrierPhaseStdev @10 :Float32;
#      # doppler standard deviation in Hz
#      dopplerStdev @11 :Float32;
#      sigId @12 :UInt8;
#
#      struct TrackingStatus {
#        # pseudorange valid
#        pseudorangeValid @0 :Bool;
#        # carrier phase valid
#        carrierPhaseValid @1 :Bool;
#        # half cycle valid
#        halfCycleValid @2 :Bool;
#        # half sycle subtracted from phase
#        halfCycleSubtracted @3 :Bool;
#      }
#    }
#  }
