# CarState Message

std_msgs/Header header

openpilot/CarEvent[] events

# car speed
std_msgs/Float32 vEgo
std_msgs/Float32 aEgo
std_msgs/Float32 vEgoRaw
std_msgs/Float32 yawRate
std_msgs/Bool standstill
openpilot/WheelSpeeds wheelSpeeds

# gas pedal, 0.0-1.0
std_msgs/Float32 gas
std_msgs/Bool gasPressed

# brake pedal, 0.0-1.0
std_msgs/Float32 brake
std_msgs/Bool brakePressed
std_msgs/Bool brakeLights

# steering wheel
std_msgs/Float32 steeringAngle
std_msgs/Float32 steeringRate
std_msgs/Float32 steeringTorque
std_msgs/Float32 steeringTorqueEps
std_msgs/Bool steeringPressed
std_msgs/Bool steeringRateLimited
std_msgs/Bool steerWarning
std_msgs/Bool steerError
std_msgs/Bool stockAeb
std_msgs/Bool stockFcw
std_msgs/Bool espDisabled

# cruise state
openpilot/CruiseState cruiseState

# gear
std_msgs/String gearShifter

# button presses
openpilot/ButtonEvent[] buttonEvents
std_msgs/Bool leftBlinker
std_msgs/Bool rightBlinker
std_msgs/Bool genericToggle

# lock info
std_msgs/Bool doorOpen
std_msgs/Bool seatbeltUnlatched
std_msgs/Bool canValid

# clutch (manual transmission only)
std_msgs/Bool clutchPressed

# which packets this state came from
std_msgs/UInt64[] canMonoTimes

# blindspot sensors
std_msgs/Bool leftBlindspot
std_msgs/Bool rightBlindspot



# struct CarState {
#   errorsDEPRECATED @0 :List(CarEvent.EventName);
#   events @13 :List(CarEvent);

#   # car speed
#   vEgo @1 :Float32;         # best estimate of speed
#   aEgo @16 :Float32;        # best estimate of acceleration
#   vEgoRaw @17 :Float32;     # unfiltered speed from CAN sensors
#   yawRate @22 :Float32;     # best estimate of yaw rate
#   standstill @18 :Bool;
#   wheelSpeeds @2 :WheelSpeeds;

#   # gas pedal, 0.0-1.0
#   gas @3 :Float32;        # this is user + computer
#   gasPressed @4 :Bool;    # this is user pedal only

#   # brake pedal, 0.0-1.0
#   brake @5 :Float32;      # this is user pedal only
#   brakePressed @6 :Bool;  # this is user pedal only
#   brakeLights @19 :Bool;

#   # steering wheel
#   steeringAngle @7 :Float32;       # deg
#   steeringRate @15 :Float32;       # deg/s
#   steeringTorque @8 :Float32;      # TODO: standardize units
#   steeringTorqueEps @27 :Float32;  # TODO: standardize units
#   steeringPressed @9 :Bool;        # if the user is using the steering wheel
#   steeringRateLimited @29 :Bool;   # if the torque is limited by the rate limiter
#   steerWarning @35 :Bool;          # temporary steer unavailble
#   steerError @36 :Bool;            # permanent steer error
#   stockAeb @30 :Bool;
#   stockFcw @31 :Bool;
#   espDisabled @32 :Bool;

#   # cruise state
#   cruiseState @10 :CruiseState;

#   # gear
#   gearShifter @14 :GearShifter;

#   # button presses
#   buttonEvents @11 :List(ButtonEvent);
#   leftBlinker @20 :Bool;
#   rightBlinker @21 :Bool;
#   genericToggle @23 :Bool;

#   # lock info
#   doorOpen @24 :Bool;
#   seatbeltUnlatched @25 :Bool;
#   canValid @26 :Bool;

#   # clutch (manual transmission only)
#   clutchPressed @28 :Bool;

#   # which packets this state came from
#   canMonoTimes @12: List(UInt64);

#   # blindspot sensors
#   leftBlindspot @33 :Bool; # Is there something blocking the left lane change
#   rightBlindspot @34 :Bool; # Is there something blocking the right lane change

#   struct WheelSpeeds {
#     # optional wheel speeds
#     fl @0 :Float32;
#     fr @1 :Float32;
#     rl @2 :Float32;
#     rr @3 :Float32;
#   }

#   struct CruiseState {
#     enabled @0 :Bool;
#     speed @1 :Float32;
#     available @2 :Bool;
#     speedOffset @3 :Float32;
#     standstill @4 :Bool;
#     nonAdaptive @5 :Bool;
#   }

#   enum GearShifter {
#     unknown @0;
#     park @1;
#     drive @2;
#     neutral @3;
#     reverse @4;
#     sport @5;
#     low @6;
#     brake @7;
#     eco @8;
#     manumatic @9;
#   }

#   # send on change
#   struct ButtonEvent {
#     pressed @0 :Bool;
#     type @1 :Type;

#     enum Type {
#       unknown @0;
#       leftBlinker @1;
#       rightBlinker @2;
#       accelCruise @3;
#       decelCruise @4;
#       cancel @5;
#       altButton1 @6;
#       altButton2 @7;
#       altButton3 @8;
#       setCruise @9;
#       resumeCruise @10;
#       gapAdjustCruise @11;
#     }
#   }
# }
