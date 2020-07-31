# InitData Message

std_msgs/Header header

std_msgs/String[] kernelArgs
std_msgs/String kernelVersion

std_msgs/String gctx
std_msgs/String dongleId

std_msgs/String deviceType
std_msgs/String version
std_msgs/String gitCommit
std_msgs/String gitBranch
std_msgs/String gitRemote

openpilot/AndroidBuildInfo androidBuildInfo
openpilot/AndroidSensor[] androidSensors
openpilot/Map androidProperties
openpilot/ChffrAndroidExtra chffrAndroidExtra
openpilot/IosBuildInfo iosBuildInfo

openpilot/PandaInfo pandaInfo

std_msgs/Bool dirty
std_msgs/Bool passive
openpilot/Map params



# struct InitData {
#   kernelArgs @0 :List(Text);
#   kernelVersion @15 :Text;

#   gctx @1 :Text;
#   dongleId @2 :Text;

#   deviceType @3 :DeviceType;
#   version @4 :Text;
#   gitCommit @10 :Text;
#   gitBranch @11 :Text;
#   gitRemote @13 :Text;

#   androidBuildInfo @5 :AndroidBuildInfo;
#   androidSensors @6 :List(AndroidSensor);
#   androidProperties @16 :Map(Text, Text);
#   chffrAndroidExtra @7 :ChffrAndroidExtra;
#   iosBuildInfo @14 :IosBuildInfo;

#   pandaInfo @8 :PandaInfo;

#   dirty @9 :Bool;
#   passive @12 :Bool;
#   params @17 :Map(Text, Text);

#   enum DeviceType {
#     unknown @0;
#     neo @1;
#     chffrAndroid @2;
#     chffrIos @3;
#   }

#   struct AndroidBuildInfo {
#     board @0 :Text;
#     bootloader @1 :Text;
#     brand @2 :Text;
#     device @3 :Text;
#     display @4 :Text;
#     fingerprint @5 :Text;
#     hardware @6 :Text;
#     host @7 :Text;
#     id @8 :Text;
#     manufacturer @9 :Text;
#     model @10 :Text;
#     product @11 :Text;
#     radioVersion @12 :Text;
#     serial @13 :Text;
#     supportedAbis @14 :List(Text);
#     tags @15 :Text;
#     time @16 :Int64;
#     type @17 :Text;
#     user @18 :Text;

#     versionCodename @19 :Text;
#     versionRelease @20 :Text;
#     versionSdk @21 :Int32;
#     versionSecurityPatch @22 :Text;
#   }

#   struct AndroidSensor {
#     id @0 :Int32;
#     name @1 :Text;
#     vendor @2 :Text;
#     version @3 :Int32;
#     handle @4 :Int32;
#     type @5 :Int32;
#     maxRange @6 :Float32;
#     resolution @7 :Float32;
#     power @8 :Float32;
#     minDelay @9 :Int32;
#     fifoReservedEventCount @10 :UInt32;
#     fifoMaxEventCount @11 :UInt32;
#     stringType @12 :Text;
#     maxDelay @13 :Int32;
#   }

#   struct ChffrAndroidExtra {
#     allCameraCharacteristics @0 :Map(Text, Text);
#   }

#   struct IosBuildInfo {
#     appVersion @0 :Text;
#     appBuild @1 :UInt32;
#     osVersion @2 :Text;
#     deviceModel @3 :Text;
#   }

#   struct PandaInfo {
#     hasPanda @0 :Bool;
#     dongleId @1 :Text;
#     stVersion @2 :Text;
#     espVersion @3 :Text;
#   }
# }
