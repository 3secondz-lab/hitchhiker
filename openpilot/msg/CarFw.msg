# CarFw Message

std_msgs/String ecu
std_msgs/String fwVersion
std_msgs/UInt32 address
std_msgs/UInt8 subAddress



#  struct CarFw {
#    ecu @0 :Ecu;
#    fwVersion @1 :Data;
#    address @2: UInt32;
#    subAddress @3: UInt8;
#  }
#
#  enum Ecu {
#    eps @0;
#    esp @1;
#    fwdRadar @2;
#    fwdCamera @3;
#    engine @4;
#    unknown @5;
#    transmission @8; # Transmission Control Module
#    srs @9; # airbag
#    gateway @10; # can gateway
#    hud @11; # heads up display
#    combinationMeter @12; # instrument cluster
#
#    # Toyota only
#    dsu @6;
#    apgs @7;
#
#    # Honda only
#    vsa @13; # Vehicle Stability Assist
#    programmedFuelInjection @14;
#    electricBrakeBooster @15;
#    shiftByWire @16;
#  }
