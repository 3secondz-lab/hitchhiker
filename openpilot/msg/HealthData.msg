# HealthData Message

std_msgs/Header header

# from can health
std_msgs/UInt32 voltage
std_msgs/UInt32 current
std_msgs/Bool ignitionLine
std_msgs/Bool controlsAllowed
std_msgs/Bool gasInterceptorDetected

std_msgs/Bool hasGps
std_msgs/UInt32 canSendErrs
std_msgs/UInt32 canFwdErrs
std_msgs/UInt32 canRxErrs
std_msgs/UInt32 gmlanSendErrs
std_msgs/String hwType
std_msgs/UInt16 fanSpeedRpm
std_msgs/String usbPowerMode
std_msgs/Bool ignitionCan
std_msgs/String safetyModel
std_msgs/String faultStatus
std_msgs/Bool powerSaveEnabled
std_msgs/UInt32 uptime
std_msgs/String[] faults



# struct HealthData {
#   # from can health
#   voltage @0 :UInt32;
#   current @1 :UInt32;
#   ignitionLine @2 :Bool;
#   controlsAllowed @3 :Bool;
#   gasInterceptorDetected @4 :Bool;
#   startedSignalDetectedDeprecated @5 :Bool;
#   hasGps @6 :Bool;
#   canSendErrs @7 :UInt32;
#   canFwdErrs @8 :UInt32;
#   canRxErrs @19 :UInt32;
#   gmlanSendErrs @9 :UInt32;
#   hwType @10 :HwType;
#   fanSpeedRpm @11 :UInt16;
#   usbPowerMode @12 :UsbPowerMode;
#   ignitionCan @13 :Bool;
#   safetyModel @14 :Car.CarParams.SafetyModel;
#   faultStatus @15 :FaultStatus;
#   powerSaveEnabled @16 :Bool;
#   uptime @17 :UInt32;
#   faults @18 :List(FaultType);

#   enum FaultStatus {
#     none @0;
#     faultTemp @1;
#     faultPerm @2;
#   }

#   enum FaultType {
#     relayMalfunction @0;
#     unusedInterruptHandled @1;
#     interruptRateCan1 @2;
#     interruptRateCan2 @3;
#     interruptRateCan3 @4;
#     interruptRateTach @5;
#     interruptRateGmlan @6;
#     interruptRateInterrupts @7;
#     interruptRateSpiDma @8;
#     interruptRateSpiCs @9;
#     interruptRateUart1 @10;
#     interruptRateUart2 @11;
#     interruptRateUart3 @12;
#     interruptRateUart5 @13;
#     interruptRateUartDma @14;
#     interruptRateUsb @15;
#     interruptRateTim1 @16;
#     interruptRateTim3 @17;
#     registerDivergent @18;
#     interruptRateKlineInit @19;
#     # Update max fault type in boardd when adding faults
#   }

#   enum HwType {
#     unknown @0;
#     whitePanda @1;
#     greyPanda @2;
#     blackPanda @3;
#     pedal @4;
#     uno @5;
#     dos @6;
#   }

#   enum UsbPowerMode {
#     none @0;
#     client @1;
#     cdp @2;
#     dcp @3;
#   }
# }
