# LidarPts Message

std_msgs/Header header

std_msgs/UInt16[] r
std_msgs/UInt16[] theta
std_msgs/UInt8[] reflect

# For storing out of file.
std_msgs/UInt64 idx

# For storing in file
std_msgs/Byte[] pkt



# struct LidarPts {
#   r @0 :List(UInt16);        # uint16   m*500.0
#   theta @1 :List(UInt16);    # uint16 deg*100.0
#   reflect @2 :List(UInt8);   # uint8      0-255

#   # For storing out of file.
#   idx @3 :UInt64;

#   # For storing in file
#   pkt @4 :Data;
# }
