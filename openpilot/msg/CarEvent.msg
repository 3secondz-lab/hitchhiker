# CarEvent Message

std_msgs/Header header

std_msgs/String name
std_msgs/Bool enable
std_msgs/Bool noEntry
std_msgs/Bool warning
std_msgs/Bool userDisable
std_msgs/Bool softDisable
std_msgs/Bool immediateDisable
std_msgs/Bool preEnable
std_msgs/Bool permanent



# struct CarEvent @0x9b1657f34caf3ad3 {
#   name @0 :EventName;
#   enable @1 :Bool;
#   noEntry @2 :Bool;
#   warning @3 :Bool;   # alerts presented only when  enabled or soft disabling
#   userDisable @4 :Bool;
#   softDisable @5 :Bool;
#   immediateDisable @6 :Bool;
#   preEnable @7 :Bool;
#   permanent @8 :Bool; # alerts presented regardless of openpilot state

#   enum EventName @0xbaa8c5d505f727de {
#     # TODO: copy from error list
#     canError @0;
#     steerUnavailable @1;
#     brakeUnavailable @2;
#     gasUnavailable @3;
#     wrongGear @4;
#     doorOpen @5;
#     seatbeltNotLatched @6;
#     espDisabled @7;
#     wrongCarMode @8;
#     steerTempUnavailable @9;
#     reverseGear @10;
#     buttonCancel @11;
#     buttonEnable @12;
#     pedalPressed @13;
#     cruiseDisabled @14;
#     radarCanError @15;
#     dataNeededDEPRECATED @16;
#     speedTooLow @17;
#     outOfSpace @18;
#     overheat @19;
#     calibrationIncomplete @20;
#     calibrationInvalid @21;
#     controlsMismatch @22;
#     pcmEnable @23;
#     pcmDisable @24;
#     noTarget @25;
#     radarFault @26;
#     modelCommIssueDEPRECATED @27;
#     brakeHold @28;
#     parkBrake @29;
#     manualRestart @30;
#     lowSpeedLockout @31;
#     plannerError @32;
#     ipasOverrideDEPRECATED @33;
#     debugAlert @34;
#     steerTempUnavailableMute @35;
#     resumeRequired @36;
#     preDriverDistracted @37;
#     promptDriverDistracted @38;
#     driverDistracted @39;
#     geofenceDEPRECATED @40;
#     driverMonitorOnDEPRECATED @41;
#     driverMonitorOffDEPRECATED @42;
#     preDriverUnresponsive @43;
#     promptDriverUnresponsive @44;
#     driverUnresponsive @45;
#     belowSteerSpeed @46;
#     calibrationProgressDEPRECATED @47;
#     lowBattery @48;
#     invalidGiraffeHondaDEPRECATED @49;
#     vehicleModelInvalid @50;
#     controlsFailed @51;
#     sensorDataInvalid @52;
#     commIssue @53;
#     tooDistracted @54;
#     posenetInvalid @55;
#     soundsUnavailable @56;
#     preLaneChangeLeft @57;
#     preLaneChangeRight @58;
#     laneChange @59;
#     invalidGiraffeToyota @60;
#     internetConnectivityNeeded @61;
#     communityFeatureDisallowed @62;
#     lowMemory @63;
#     stockAeb @64;
#     ldw @65;
#     carUnrecognized @66;
#     radarCommIssue @67;
#     driverMonitorLowAcc @68;
#     invalidLkasSetting @69;
#     speedTooHigh @70;
#     laneChangeBlocked @71;
#     relayMalfunction @72;
#     gasPressed @73;
#     stockFcw @74;
#     startup @75;
#     startupNoCar @76;
#     startupNoControl @77;
#     startupMaster @78;
#     fcw @79;
#     steerSaturated @80;
#     whitePandaUnsupported @81;
#     startupWhitePanda @82;
#     canErrorPersistentDEPRECATED @83;
#     belowEngageSpeed @84;
#     noGps @85;
#     focusRecoverActive @86;
#     wrongCruiseMode @87;
#     neosUpdateRequired @88;
#     modeldLagging @89;
#   }
# }
