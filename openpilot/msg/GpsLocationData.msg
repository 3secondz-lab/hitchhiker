# GpsLocationData Message

std_msgs/Header header

# Contains GpsLocationFlags bits.
std_msgs/UInt16 flags

# Represents latitude in degrees.
std_msgs/Float64 latitude

# Represents longitude in degrees.
std_msgs/Float64 longitude

# Represents altitude in meters above the WGS 84 reference ellipsoid.
std_msgs/Float64 altitude

# Represents speed in meters per second.
std_msgs/Float32 speed

# Represents heading in degrees.
std_msgs/Float32 bearing

# Represents expected accuracy in meters. (presumably 1 sigma?)
std_msgs/Float32 accuracy

# Timestamp for the location fix.
# Milliseconds since January 1, 1970.
std_msgs/Int64 timestamp

std_msgs/UInt64 source

# Represents NED velocity in m/s.
std_msgs/Float32[] vNED

# Represents expected vertical accuracy in meters. (presumably 1 sigma?)
std_msgs/Float32 verticalAccuracy

# Represents bearing accuracy in degrees. (presumably 1 sigma?)
std_msgs/Float32 bearingAccuracy

# Represents velocity accuracy in m/s. (presumably 1 sigma?)
std_msgs/Float32 speedAccuracy



# # android struct GpsLocation
# struct GpsLocationData {
#   # Contains GpsLocationFlags bits.
#   flags @0 :UInt16;

#   # Represents latitude in degrees.
#   latitude @1 :Float64;

#   # Represents longitude in degrees.
#   longitude @2 :Float64;

#   # Represents altitude in meters above the WGS 84 reference ellipsoid.
#   altitude @3 :Float64;

#   # Represents speed in meters per second.
#   speed @4 :Float32;

#   # Represents heading in degrees.
#   bearing @5 :Float32;

#   # Represents expected accuracy in meters. (presumably 1 sigma?)
#   accuracy @6 :Float32;

#   # Timestamp for the location fix.
#   # Milliseconds since January 1, 1970.
#   timestamp @7 :Int64;

#   source @8 :SensorSource;

#   # Represents NED velocity in m/s.
#   vNED @9 :List(Float32);

#   # Represents expected vertical accuracy in meters. (presumably 1 sigma?)
#   verticalAccuracy @10 :Float32;

#   # Represents bearing accuracy in degrees. (presumably 1 sigma?)
#   bearingAccuracy @11 :Float32;

#   # Represents velocity accuracy in m/s. (presumably 1 sigma?)
#   speedAccuracy @12 :Float32;

#   enum SensorSource {
#     android @0;
#     iOS @1;
#     car @2;
#     velodyne @3;  # Velodyne IMU
#     fusion @4;
#     external @5;
#     ublox @6;
#     trimble @7;
#   }
# }
