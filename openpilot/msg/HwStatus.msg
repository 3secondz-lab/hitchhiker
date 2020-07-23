# HwStatus Message

std_msgs/UInt16 noisePerMS
std_msgs/UInt16 agcCnt
std_msgs/String aStatus
std_msgs/String aPower
std_msgs/UInt8 jamInd



#  struct HwStatus {
#    noisePerMS @0 :UInt16;
#    agcCnt @1 :UInt16;
#    aStatus @2 :AntennaSupervisorState;
#    aPower @3 :AntennaPowerStatus;
#    jamInd @4 :UInt8;
#
#    enum AntennaSupervisorState {
#      init @0;
#      dontknow @1;
#      ok @2;
#      short @3;
#      open @4;
#    }
#
#    enum AntennaPowerStatus {
#      off @0;
#      on @1;
#      dontknow @2;
#    }
#  }
