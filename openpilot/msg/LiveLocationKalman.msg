# LiveLocationKalman Message

std_msgs/Header header

# More info on reference frames:
# https://github.com/commaai/openpilot/tree/master/common/transformations

openpilot/MeasurementKalman positionECEF
openpilot/MeasurementKalman positionGeodetic
openpilot/MeasurementKalman velocityECEF
openpilot/MeasurementKalman velocityNED
openpilot/MeasurementKalman velocityDevice
openpilot/MeasurementKalman accelerationDevice

# These angles are all eulers and roll, pitch, yaw
# orientationECEF transforms to rot matrix: ecef_from_device
openpilot/MeasurementKalman orientationECEF
openpilot/MeasurementKalman calibratedOrientationECEF
openpilot/MeasurementKalman orientationNED
openpilot/MeasurementKalman angularVelocityDevice

# orientationNEDCalibrated transforms to rot matrix: NED_from_calibrated
openpilot/MeasurementKalman orientationNEDCalibrated

# Calibrated frame is simply device frame
# aligned with the vehicle
openpilot/MeasurementKalman velocityCalibrated
openpilot/MeasurementKalman accelerationCalibrated
openpilot/MeasurementKalman angularVelocityCalibrated

std_msgs/Int32 gpsWeek
std_msgs/Float64 gpsTimeOfWeek
std_msgs/String status
std_msgs/Int64 unixTimestampMillis
std_msgs/Bool inputsOK
std_msgs/Bool posenetOK
std_msgs/Bool gpsOK
std_msgs/Bool sensorsOK



# struct LiveLocationKalman {

#   # More info on reference frames:
#   # https://github.com/commaai/openpilot/tree/master/common/transformations

#   positionECEF @0 : Measurement;
#   positionGeodetic @1 : Measurement;
#   velocityECEF @2 : Measurement;
#   velocityNED @3 : Measurement;
#   velocityDevice @4 : Measurement;
#   accelerationDevice @5: Measurement;


#   # These angles are all eulers and roll, pitch, yaw
#   # orientationECEF transforms to rot matrix: ecef_from_device
#   orientationECEF @6 : Measurement;
#   calibratedOrientationECEF @20 : Measurement;
#   orientationNED @7 : Measurement;
#   angularVelocityDevice @8 : Measurement;

#   # orientationNEDCalibrated transforms to rot matrix: NED_from_calibrated
#   orientationNEDCalibrated @9 : Measurement;

#   # Calibrated frame is simply device frame
#   # aligned with the vehicle
#   velocityCalibrated @10 : Measurement;
#   accelerationCalibrated @11 : Measurement;
#   angularVelocityCalibrated @12 : Measurement;

#   gpsWeek @13 :Int32;
#   gpsTimeOfWeek @14 :Float64;
#   status @15 :Status;
#   unixTimestampMillis @16 :Int64;
#   inputsOK @17 :Bool = true;
#   posenetOK @18 :Bool = true;
#   gpsOK @19 :Bool = true;
#   sensorsOK @21 :Bool = true;

#   enum Status {
#     uninitialized @0;
#     uncalibrated @1;
#     valid @2;
#   }

#   struct Measurement {
#     value @0 : List(Float64);
#     std @1 : List(Float64);
#     valid @2 : Bool;
#   }
# }
