# LateralLQRTuning Message

std_msgs/Float32 scale
std_msgs/Float32 ki
std_msgs/Float32 dcGain

# State space system
std_msgs/Float32[] a
std_msgs/Float32[] b
std_msgs/Float32[] c

std_msgs/Float32[] k
std_msgs/Float32[] l



#  struct LateralLQRTuning {
#    scale @0 :Float32;
#    ki @1 :Float32;
#    dcGain @2 :Float32;
#
#    # State space system
#    a @3 :List(Float32);
#    b @4 :List(Float32);
#    c @5 :List(Float32);
#
#    k @6 :List(Float32);  # LQR gain
#    l @7 :List(Float32);  # Kalman gain
#  }
