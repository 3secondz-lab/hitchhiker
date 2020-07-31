# ProcLog Message

std_msgs/Header header

openpilot/CPUTimes[] cpuTimes
openpilot/Mem mem
openpilot/Process[] procs



# struct ProcLog {
#   cpuTimes @0 :List(CPUTimes);
#   mem @1 :Mem;
#   procs @2 :List(Process);

#   struct Process {
#     pid @0 :Int32;
#     name @1 :Text;
#     state @2 :UInt8;
#     ppid @3 :Int32;

#     cpuUser @4 :Float32;
#     cpuSystem @5 :Float32;
#     cpuChildrenUser @6 :Float32;
#     cpuChildrenSystem @7 :Float32;
#     priority @8 :Int64;
#     nice @9 :Int32;
#     numThreads @10 :Int32;
#     startTime @11 :Float64;

#     memVms @12 :UInt64;
#     memRss @13 :UInt64;

#     processor @14 :Int32;

#     cmdline @15 :List(Text);
#     exe @16 :Text;
#   }

#   struct CPUTimes {
#     cpuNum @0 :Int64;
#     user @1 :Float32;
#     nice @2 :Float32;
#     system @3 :Float32;
#     idle @4 :Float32;
#     iowait @5 :Float32;
#     irq @6 :Float32;
#     softirq @7 :Float32;
#   }

#   struct Mem {
#     total @0 :UInt64;
#     free @1 :UInt64;
#     available @2 :UInt64;
#     buffers @3 :UInt64;
#     cached @4 :UInt64;
#     active @5 :UInt64;
#     inactive @6 :UInt64;
#     shared @7 :UInt64;
#   }

# }
