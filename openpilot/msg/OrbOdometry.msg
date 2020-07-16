# OrbOdometry Message

std_msgs/Header Header

# timing first
std_msgs/UInt64 startMonoTime
std_msgs/UInt64 endMonoTime

# fundamental matrix and error
std_msgs/Float64[] f
std_msgs/Float64 err

# number of inlier points
std_msgs/Int32 inliers

# for debug only
# indexed by endMonoTime features
# value is startMonoTime feature match
# -1 if no match
std_msgs/Int16[] matches



# struct OrbOdometry {
#   # timing first
#   startMonoTime @0 :UInt64;
#   endMonoTime @1 :UInt64;

#   # fundamental matrix and error
#   f @2: List(Float64);
#   err @3: Float64;

#   # number of inlier points
#   inliers @4: Int32;

#   # for debug only
#   # indexed by endMonoTime features
#   # value is startMonoTime feature match
#   # -1 if no match
#   matches @5: List(Int16);
# }
