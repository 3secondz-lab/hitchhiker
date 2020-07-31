# CarParams Message

std_msgs/Header header

std_msgs/String carName
std_msgs/String carFingerprint

std_msgs/Bool enableGasInterceptor
std_msgs/Bool enableCruise
std_msgs/Bool enableCamera
std_msgs/Bool enableDsu
std_msgs/Bool enableApgs

std_msgs/Float32 minEnableSpeed
std_msgs/Float32 minSteerSpeed
std_msgs/String safetyModel
std_msgs/String safetyModelPassive
std_msgs/Int16 safetyParam

std_msgs/Float32[] steerMaxBP
std_msgs/Float32[] steerMaxV
std_msgs/Float32[] gasMaxBP
std_msgs/Float32[] gasMaxV
std_msgs/Float32[] brakeMaxBP
std_msgs/Float32[] brakeMaxV

# things about the car in the manual
std_msgs/Float32 mass
std_msgs/Float32 wheelbase
std_msgs/Float32 centerToFront
std_msgs/Float32 steerRatio
std_msgs/Float32 steerRatioRear

# things we can derive
std_msgs/Float32 rotationalInertia
std_msgs/Float32 tireStiffnessFront
std_msgs/Float32 tireStiffnessRear

openpilot/LongitudinalPIDTuning longitudinalTuning
openpilot/LateralParams lateralParams

openpilot/LateralPIDTuning pid
openpilot/LateralINDITuning indi
openpilot/LateralLQRTuning lqr

std_msgs/Bool steerLimitAlert
std_msgs/Float32 steerLimitTimer

std_msgs/Float32 vEgoStopping
std_msgs/Bool directAccelControl
std_msgs/Bool stoppingControl
std_msgs/Float32 startAccel
std_msgs/Float32 steerRateCost
std_msgs/String steerControlType
std_msgs/Bool radarOffCan

std_msgs/Float32 steerActuatorDelay
std_msgs/Bool openpilotLongitudinalControl
std_msgs/String carVin
std_msgs/Bool isPandaBlack
std_msgs/Bool dashcamOnly
std_msgs/String transmissionType
openpilot/CarFw[] carFw
std_msgs/Float32 radarTimeStep
std_msgs/Bool communityFeature
std_msgs/String fingerprintSource
std_msgs/String networkLocation



# struct CarParams {
#   carName @0 :Text;
#   carFingerprint @1 :Text;

#   enableGasInterceptor @2 :Bool;
#   enableCruise @3 :Bool;
#   enableCamera @4 :Bool;
#   enableDsu @5 :Bool; # driving support unit
#   enableApgs @6 :Bool; # advanced parking guidance system

#   minEnableSpeed @7 :Float32;
#   minSteerSpeed @8 :Float32;
#   safetyModel @9 :SafetyModel;
#   safetyModelPassive @42 :SafetyModel = silent;
#   safetyParam @10 :Int16;

#   steerMaxBP @11 :List(Float32);
#   steerMaxV @12 :List(Float32);
#   gasMaxBP @13 :List(Float32);
#   gasMaxV @14 :List(Float32);
#   brakeMaxBP @15 :List(Float32);
#   brakeMaxV @16 :List(Float32);

#   # things about the car in the manual
#   mass @17 :Float32;             # [kg] running weight
#   wheelbase @18 :Float32;        # [m] distance from rear to front axle
#   centerToFront @19 :Float32;   # [m] GC distance to front axle
#   steerRatio @20 :Float32;       # [] ratio between front wheels and steering wheel angles
#   steerRatioRear @21 :Float32;  # [] rear steering ratio wrt front steering (usually 0)

#   # things we can derive
#   rotationalInertia @22 :Float32;    # [kg*m2] body rotational inertia
#   tireStiffnessFront @23 :Float32;   # [N/rad] front tire coeff of stiff
#   tireStiffnessRear @24 :Float32;    # [N/rad] rear tire coeff of stiff

#   longitudinalTuning @25 :LongitudinalPIDTuning;
#   lateralParams @48 :LateralParams;
#   lateralTuning :union {
#     pid @26 :LateralPIDTuning;
#     indi @27 :LateralINDITuning;
#     lqr @40 :LateralLQRTuning;
#   }

#   steerLimitAlert @28 :Bool;
#   steerLimitTimer @47 :Float32;  # time before steerLimitAlert is issued

#   vEgoStopping @29 :Float32; # Speed at which the car goes into stopping state
#   directAccelControl @30 :Bool; # Does the car have direct accel control or just gas/brake
#   stoppingControl @31 :Bool; # Does the car allows full control even at lows speeds when stopping
#   startAccel @32 :Float32; # Required acceleraton to overcome creep braking
#   steerRateCost @33 :Float32; # Lateral MPC cost on steering rate
#   steerControlType @34 :SteerControlType;
#   radarOffCan @35 :Bool; # True when radar objects aren't visible on CAN

#   steerActuatorDelay @36 :Float32; # Steering wheel actuator delay in seconds
#   openpilotLongitudinalControl @37 :Bool; # is openpilot doing the longitudinal control?
#   carVin @38 :Text; # VIN number queried during fingerprinting
#   isPandaBlack @39: Bool;
#   dashcamOnly @41: Bool;
#   transmissionType @43 :TransmissionType;
#   carFw @44 :List(CarFw);
#   radarTimeStep @45: Float32 = 0.05;  # time delta between radar updates, 20Hz is very standard
#   communityFeature @46: Bool;  # true if a community maintained feature is detected
#   fingerprintSource @49: FingerprintSource;
#   networkLocation @50 :NetworkLocation;  # Where Panda/C2 is integrated into the car's CAN network

#   struct LateralParams {
#     torqueBP @0 :List(Int32);
#     torqueV @1 :List(Int32);
#   }

#   struct LateralPIDTuning {
#     kpBP @0 :List(Float32);
#     kpV @1 :List(Float32);
#     kiBP @2 :List(Float32);
#     kiV @3 :List(Float32);
#     kf @4 :Float32;
#   }

#   struct LongitudinalPIDTuning {
#     kpBP @0 :List(Float32);
#     kpV @1 :List(Float32);
#     kiBP @2 :List(Float32);
#     kiV @3 :List(Float32);
#     deadzoneBP @4 :List(Float32);
#     deadzoneV @5 :List(Float32);
#   }

#   struct LateralINDITuning {
#     outerLoopGain @0 :Float32;
#     innerLoopGain @1 :Float32;
#     timeConstant @2 :Float32;
#     actuatorEffectiveness @3 :Float32;
#   }

#   struct LateralLQRTuning {
#     scale @0 :Float32;
#     ki @1 :Float32;
#     dcGain @2 :Float32;

#     # State space system
#     a @3 :List(Float32);
#     b @4 :List(Float32);
#     c @5 :List(Float32);

#     k @6 :List(Float32);  # LQR gain
#     l @7 :List(Float32);  # Kalman gain
#   }

#   enum SafetyModel {
#     silent @0;
#     hondaNidec @1;
#     toyota @2;
#     elm327 @3;
#     gm @4;
#     hondaBoschGiraffe @5;
#     ford @6;
#     cadillac @7;
#     hyundai @8;
#     chrysler @9;
#     tesla @10;
#     subaru @11;
#     gmPassive @12;
#     mazda @13;
#     nissan @14;
#     volkswagen @15;
#     toyotaIpas @16;
#     allOutput @17;
#     gmAscm @18;
#     noOutput @19;  # like silent but without silent CAN TXs
#     hondaBoschHarness @20;
#     volkswagenPq @21;
#     subaruLegacy @22;  # pre-Global platform
#     hyundaiLegacy @23;
#   }

#   enum SteerControlType {
#     torque @0;
#     angle @1;
#   }

#   enum TransmissionType {
#     unknown @0;
#     automatic @1;  # Traditional auto, including DSG
#     manual @2;  # True "stick shift" only
#     direct @3;  # Electric vehicle or other direct drive
#   }

#   struct CarFw {
#     ecu @0 :Ecu;
#     fwVersion @1 :Data;
#     address @2: UInt32;
#     subAddress @3: UInt8;
#   }

#   enum Ecu {
#     eps @0;
#     esp @1;
#     fwdRadar @2;
#     fwdCamera @3;
#     engine @4;
#     unknown @5;
#     transmission @8; # Transmission Control Module
#     srs @9; # airbag
#     gateway @10; # can gateway
#     hud @11; # heads up display
#     combinationMeter @12; # instrument cluster

#     # Toyota only
#     dsu @6;
#     apgs @7;

#     # Honda only
#     vsa @13; # Vehicle Stability Assist
#     programmedFuelInjection @14;
#     electricBrakeBooster @15;
#     shiftByWire @16;
#   }

#   enum FingerprintSource {
#     can @0;
#     fw @1;
#     fixed @2;
#   }

#   enum NetworkLocation {
#     fwdCamera @0;  # Standard/default integration at LKAS camera
#     gateway @1;    # Integration at vehicle's CAN gateway
#   }
# }
