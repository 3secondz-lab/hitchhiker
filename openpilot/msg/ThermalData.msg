# ThermalData Message

std_msgs/Header header

std_msgs/UInt16 cpu0
std_msgs/UInt16 cpu1
std_msgs/UInt16 cpu2
std_msgs/UInt16 cpu3
std_msgs/UInt16 mem
std_msgs/UInt16 gpu
std_msgs/UInt32 bat
std_msgs/UInt16 pa0

# not thermal
std_msgs/Float32 freeSpace
std_msgs/Int16 batteryPercent
std_msgs/String batteryStatus
std_msgs/Int32 batteryCurrent
std_msgs/Int32 batteryVoltage
std_msgs/Bool usbOnline
std_msgs/UInt64 networkType
std_msgs/UInt32 offroadPowerUsage
std_msgs/UInt64 networkStrength

std_msgs/UInt16 fanSpeed
std_msgs/Bool started
std_msgs/UInt64 startedTs

std_msgs/UInt64 thermalStatus
std_msgs/Bool chargingError
std_msgs/Bool chargingDisabled

std_msgs/Int8 memUsedPercent
std_msgs/Int8 cpuPerc



# struct ThermalData {
#   cpu0 @0 :UInt16;
#   cpu1 @1 :UInt16;
#   cpu2 @2 :UInt16;
#   cpu3 @3 :UInt16;
#   mem @4 :UInt16;
#   gpu @5 :UInt16;
#   bat @6 :UInt32;
#   pa0 @21 :UInt16;

#   # not thermal
#   freeSpace @7 :Float32;
#   batteryPercent @8 :Int16;
#   batteryStatus @9 :Text;
#   batteryCurrent @15 :Int32;
#   batteryVoltage @16 :Int32;
#   usbOnline @12 :Bool;
#   networkType @22 :NetworkType;
#   offroadPowerUsage @23 :UInt32;  # Power usage since going offroad in uWh
#   networkStrength @24 :NetworkStrength;

#   fanSpeed @10 :UInt16;
#   started @11 :Bool;
#   startedTs @13 :UInt64;

#   thermalStatus @14 :ThermalStatus;
#   chargingError @17 :Bool;
#   chargingDisabled @18 :Bool;

#   memUsedPercent @19 :Int8;
#   cpuPerc @20 :Int8;

#   enum ThermalStatus {
#     green @0;   # all processes run
#     yellow @1;  # critical processes run (kill uploader), engage still allowed
#     red @2;     # no engage, will disengage
#     danger @3;  # immediate process shutdown
#   }

#   enum NetworkType {
#     none @0;
#     wifi @1;
#     cell2G @2;
#     cell3G @3;
#     cell4G @4;
#     cell5G @5;
#   }

#   enum NetworkStrength {
#     unknown @0;
#     poor @1;
#     moderate @2;
#     good @3;
#     great @4;
#   }
# }
