# Actuators Message

# range from 0.0 - 1.0
std_msgs/Float32 gas
std_msgs/Float32 brake

# range from -1.0 - 1.0
std_msgs/Float32 steer
std_msgs/Float32 steerAngle



#  struct Actuators {
#    # range from 0.0 - 1.0
#    gas @0: Float32;
#    brake @1: Float32;
#    # range from -1.0 - 1.0
#    steer @2: Float32;
#    steerAngle @3: Float32;
#  }
