# OrbKeyFrame Message

std_msgs/Header header

# this is a globally unique id for the KeyFrame
std_msgs/UInt64 id

# this is the location of the KeyFrame
openpilot/ECEFPoint pos

# these are the features in the world
# len(dpos) == len(descriptors) * 32
openpilot/ECEFPoint[] dpos
std_msgs/String descriptors



# struct OrbKeyFrame {
#   # this is a globally unique id for the KeyFrame
#   id @0: UInt64;

#   # this is the location of the KeyFrame
#   pos @1: ECEFPoint;

#   # these are the features in the world
#   # len(dpos) == len(descriptors) * 32
#   dpos @2 :List(ECEFPoint);
#   descriptors @3 :Data;
# }
