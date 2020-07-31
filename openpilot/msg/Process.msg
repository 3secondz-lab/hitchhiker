# Process Message

std_msgs/Int32 pid
std_msgs/String name
std_msgs/UInt8 state
std_msgs/Int32 ppid

std_msgs/Float32 cpuUser
std_msgs/Float32 cpuSystem
std_msgs/Float32 cpuChildrenUser
std_msgs/Float32 cpuChildrenSystem
std_msgs/Int64 priority
std_msgs/Int32 nice
std_msgs/Int32 numThreads
std_msgs/Float64 startTime

std_msgs/UInt64 memVms
std_msgs/UInt64 memRss

std_msgs/Int32 processor

std_msgs/String[] cmdline
std_msgs/String exe




#  struct Process {
#    pid @0 :Int32;
#    name @1 :Text;
#    state @2 :UInt8;
#    ppid @3 :Int32;
#
#    cpuUser @4 :Float32;
#    cpuSystem @5 :Float32;
#    cpuChildrenUser @6 :Float32;
#    cpuChildrenSystem @7 :Float32;
#    priority @8 :Int64;
#    nice @9 :Int32;
#    numThreads @10 :Int32;
#    startTime @11 :Float64;
#
#    memVms @12 :UInt64;
#    memRss @13 :UInt64;
#
#    processor @14 :Int32;
#
#    cmdline @15 :List(Text);
#    exe @16 :Text;
#  }
