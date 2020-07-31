# ControlsState Message

std_msgs/Header header

std_msgs/UInt64[] canMonoTimes
std_msgs/UInt64 planMonoTime
std_msgs/UInt64 pathPlanMonoTime

std_msgs/String state
std_msgs/Float32 vEgo
std_msgs/Float32 vEgoRaw
std_msgs/String longControlState
std_msgs/Float32 vPid
std_msgs/Float32 vTargetLead
std_msgs/Float32 upAccelCmd
std_msgs/Float32 uiAccelCmd
std_msgs/Float32 ufAccelCmd
std_msgs/Float32 aTarget
std_msgs/Float32 jerkFactor
std_msgs/Float32 angleSteers
std_msgs/Float32 angleSteersDes
std_msgs/Float32 curvature
std_msgs/Float32 cumLagMs
std_msgs/UInt64 startMonoTime
std_msgs/Bool mapValid
std_msgs/Bool forceDecel

std_msgs/Bool enabled
std_msgs/Bool active
std_msgs/Bool steerOverride

std_msgs/Float32 vCruise

std_msgs/Bool rearViewCam
std_msgs/String alertText1
std_msgs/String alertText2
std_msgs/String alertStatus
std_msgs/String alertSize
std_msgs/Float32 alertBlinkingRate
std_msgs/String alertType
std_msgs/String alertSound
std_msgs/Float32 awarenessStatus
std_msgs/Bool gpsPlannerActive
std_msgs/Bool engageable
std_msgs/Bool driverMonitoringOn

# maps
std_msgs/Float32 vCurvature
std_msgs/Bool decelForTurn

std_msgs/Bool decelForModel
std_msgs/UInt32 canErrorCounter

openpilot/LateralINDIState indiState
openpilot/LateralPIDState pidState
openpilot/LateralLQRState lqrState



# struct ControlsState @0x97ff69c53601abf1 {
#   canMonoTimeDEPRECATED @16 :UInt64;
#   canMonoTimes @21 :List(UInt64);
#   radarStateMonoTimeDEPRECATED @17 :UInt64;
#   mdMonoTimeDEPRECATED @18 :UInt64;
#   planMonoTime @28 :UInt64;
#   pathPlanMonoTime @50 :UInt64;

#   state @31 :OpenpilotState;
#   vEgo @0 :Float32;
#   vEgoRaw @32 :Float32;
#   aEgoDEPRECATED @1 :Float32;
#   longControlState @30 :LongControlState;
#   vPid @2 :Float32;
#   vTargetLead @3 :Float32;
#   upAccelCmd @4 :Float32;
#   uiAccelCmd @5 :Float32;
#   ufAccelCmd @33 :Float32;
#   yActualDEPRECATED @6 :Float32;
#   yDesDEPRECATED @7 :Float32;
#   upSteerDEPRECATED @8 :Float32;
#   uiSteerDEPRECATED @9 :Float32;
#   ufSteerDEPRECATED @34 :Float32;
#   aTargetMinDEPRECATED @10 :Float32;
#   aTargetMaxDEPRECATED @11 :Float32;
#   aTarget @35 :Float32;
#   jerkFactor @12 :Float32;
#   angleSteers @13 :Float32;     # Steering angle in degrees.
#   angleSteersDes @29 :Float32;
#   curvature @37 :Float32;       # path curvature from vehicle model
#   hudLeadDEPRECATED @14 :Int32;
#   cumLagMs @15 :Float32;
#   startMonoTime @48 :UInt64;
#   mapValid @49 :Bool;
#   forceDecel @51 :Bool;

#   enabled @19 :Bool;
#   active @36 :Bool;
#   steerOverride @20 :Bool;

#   vCruise @22 :Float32;

#   rearViewCam @23 :Bool;
#   alertText1 @24 :Text;
#   alertText2 @25 :Text;
#   alertStatus @38 :AlertStatus;
#   alertSize @39 :AlertSize;
#   alertBlinkingRate @42 :Float32;
#   alertType @44 :Text;
#   alertSoundDEPRECATED @45 :Text;
#   alertSound @56 :Car.CarControl.HUDControl.AudibleAlert;
#   awarenessStatus @26 :Float32;
#   angleModelBiasDEPRECATED @27 :Float32;
#   gpsPlannerActive @40 :Bool;
#   engageable @41 :Bool;  # can OP be engaged?
#   driverMonitoringOn @43 :Bool;

#   # maps
#   vCurvature @46 :Float32;
#   decelForTurn @47 :Bool;

#   decelForModel @54 :Bool;
#   canErrorCounter @57 :UInt32;

#   lateralControlState :union {
#     indiState @52 :LateralINDIState;
#     pidState @53 :LateralPIDState;
#     lqrState @55 :LateralLQRState;
#   }

#   enum OpenpilotState @0xdbe58b96d2d1ac61 {
#     disabled @0;
#     preEnabled @1;
#     enabled @2;
#     softDisabling @3;
#   }

#   enum LongControlState {
#     off @0;
#     pid @1;
#     stopping @2;
#     starting @3;
#   }

#   enum AlertStatus {
#     normal @0;       # low priority alert for user's convenience
#     userPrompt @1;   # mid piority alert that might require user intervention
#     critical @2;     # high priority alert that needs immediate user intervention
#   }

#   enum AlertSize {
#     none @0;    # don't display the alert
#     small @1;   # small box
#     mid @2;     # mid screen
#     full @3;    # full screen
#   }

#   struct LateralINDIState {
#     active @0 :Bool;
#     steerAngle @1 :Float32;
#     steerRate @2 :Float32;
#     steerAccel @3 :Float32;
#     rateSetPoint @4 :Float32;
#     accelSetPoint @5 :Float32;
#     accelError @6 :Float32;
#     delayedOutput @7 :Float32;
#     delta @8 :Float32;
#     output @9 :Float32;
#     saturated @10 :Bool;
#   }

#   struct LateralPIDState {
#     active @0 :Bool;
#     steerAngle @1 :Float32;
#     steerRate @2 :Float32;
#     angleError @3 :Float32;
#     p @4 :Float32;
#     i @5 :Float32;
#     f @6 :Float32;
#     output @7 :Float32;
#     saturated @8 :Bool;
#    }

#   struct LateralLQRState {
#     active @0 :Bool;
#     steerAngle @1 :Float32;
#     i @2 :Float32;
#     output @3 :Float32;
#     lqrOutput @4 :Float32;
#     saturated @5 :Bool;
#    }


# }
