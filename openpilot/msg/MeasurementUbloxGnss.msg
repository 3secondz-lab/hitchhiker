# MeasurementUbloxGnss Message

std_msgs/UInt8 svId
openpilot/TrackingStatus trackingStatus
# pseudorange in meters
std_msgs/Float64 pseudorange
# carrier phase measurement in cycles
std_msgs/Float64 carrierCycles
# doppler measurement in Hz
std_msgs/Float32 doppler
# GNSS id, 0 is gps
std_msgs/UInt8 gnssId
std_msgs/UInt8 glonassFrequencyIndex
# carrier phase locktime counter in ms
std_msgs/UInt16 locktime
# Carrier-to-noise density ratio (signal strength) in dBHz
std_msgs/UInt8 cno
# pseudorange standard deviation in meters
std_msgs/Float32 pseudorangeStdev
# carrier phase standard deviation in cycles
std_msgs/Float32 carrierPhaseStdev
# doppler standard deviation in Hz
std_msgs/Float32 dopplerStdev
std_msgs/UInt8 sigId



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
