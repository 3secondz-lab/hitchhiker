# LiveLocationData Message

std_msgs/Header header

std_msgs/UInt8 status

# 3D fix
std_msgs/Float64 lat
std_msgs/Float64 lon
std_msgs/Float64 alt

# speed
std_msgs/Float32 speed

# NED velocity components
std_msgs/Float32[] vNED

# roll, pitch, heading (x,y,z)
std_msgs/Float32 roll
std_msgs/Float32 pitch
std_msgs/Float32 heading

# what are these?
std_msgs/Float32 wanderAngle
std_msgs/Float32 trackAngle

# car frame -- https://upload.wikimedia.org/wikipedia/commons/f/f5/RPY_angles_of_cars.png

# gyro, in car frame, deg/s
std_msgs/Float32[] gyro

# accel, in car frame, m/s^2
std_msgs/Float32[] accel

openpilot/Accuracy accuracy

std_msgs/UInt64 source
# if we are fixing a location in the past
std_msgs/UInt64 fixMonoTime

std_msgs/Int32 gpsWeek
std_msgs/Float64 timeOfWeek

std_msgs/Float64[] positionECEF
std_msgs/Float32[] poseQuatECEF
std_msgs/Float32 pitchCalibration
std_msgs/Float32 yawCalibration
std_msgs/Float32[] imuFrame



# struct LiveLocationData {
#   status @0 :UInt8;

#   # 3D fix
#   lat @1 :Float64;
#   lon @2 :Float64;
#   alt @3 :Float32;     # m

#   # speed
#   speed @4 :Float32;   # m/s

#   # NED velocity components
#   vNED @5 :List(Float32);

#   # roll, pitch, heading (x,y,z)
#   roll @6 :Float32;     # WRT to center of earth?
#   pitch @7 :Float32;    # WRT to center of earth?
#   heading @8 :Float32;  # WRT to north?

#   # what are these?
#   wanderAngle @9 :Float32;
#   trackAngle @10 :Float32;

#   # car frame -- https://upload.wikimedia.org/wikipedia/commons/f/f5/RPY_angles_of_cars.png

#   # gyro, in car frame, deg/s
#   gyro @11 :List(Float32);

#   # accel, in car frame, m/s^2
#   accel @12 :List(Float32);

#   accuracy @13 :Accuracy;

#   source @14 :SensorSource;
#   # if we are fixing a location in the past
#   fixMonoTime @15 :UInt64;

#   gpsWeek @16 :Int32;
#   timeOfWeek @17 :Float64;

#   positionECEF @18 :List(Float64);
#   poseQuatECEF @19 :List(Float32);
#   pitchCalibration @20 :Float32;
#   yawCalibration @21 :Float32;
#   imuFrame @22 :List(Float32);

#   struct Accuracy {
#     pNEDError @0 :List(Float32);
#     vNEDError @1 :List(Float32);
#     rollError @2 :Float32;
#     pitchError @3 :Float32;
#     headingError @4 :Float32;
#     ellipsoidSemiMajorError @5 :Float32;
#     ellipsoidSemiMinorError @6 :Float32;
#     ellipsoidOrientationError @7 :Float32;
#   }

#   enum SensorSource {
#     applanix @0;
#     kalman @1;
#     orbslam @2;
#     timing @3;
#     dummy @4;
#   }
# }
