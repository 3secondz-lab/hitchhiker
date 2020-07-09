# SensorEventData Message

std_msgs/Header header

std_msgs/Int32 version
std_msgs/Int32 sensor
std_msgs/Int32 type
std_msgs/Int64 timestamp

openpilot/SensorVec acceleration
openpilot/SensorVec magnetic
openpilot/SensorVec orientation
openpilot/SensorVec gyro
openpilot/SensorVec pressure
openpilot/SensorVec magneticUncalibrated
openpilot/SensorVec gyroUncalibrated
std_msgs/Float32 proximity
std_msgs/Float32 light

std_msgs/UInt64 source



# struct SensorEventData {
#   version @0 :Int32;
#   sensor @1 :Int32;
#   type @2 :Int32;
#   timestamp @3 :Int64;
#   uncalibratedDEPRECATED @10 :Bool;

#   union {
#     acceleration @4 :SensorVec;
#     magnetic @5 :SensorVec;
#     orientation @6 :SensorVec;
#     gyro @7 :SensorVec;
#     pressure @9 :SensorVec;
#     magneticUncalibrated @11 :SensorVec;
#     gyroUncalibrated @12 :SensorVec;
#     proximity @13: Float32;
#     light @14: Float32;
#   }
#   source @8 :SensorSource;

#   struct SensorVec {
#     v @0 :List(Float32);
#     status @1 :Int8;
#   }

#   enum SensorSource {
#     android @0;
#     iOS @1;
#     fiber @2;
#     velodyne @3;  # Velodyne IMU
#     bno055 @4;    # Bosch accelerometer
#     lsm6ds3 @5;   # accelerometer (c2)
#     bmp280 @6;    # barometer (c2)
#     mmc3416x @7;  # magnetometer (c2)
#   }
# }
