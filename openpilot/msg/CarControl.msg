# CarControl Message

std_msgs/Header header

# must be true for any actuator commands to work
std_msgs/Bool enabled
std_msgs/Bool active

openpilot/Actuators actuators

openpilot/CruiseControl cruiseControl
openpilot/HUDControl hudControl



# struct CarControl {
#   # must be true for any actuator commands to work
#   enabled @0 :Bool;
#   active @7 :Bool;

#   gasDEPRECATED @1 :Float32;
#   brakeDEPRECATED @2 :Float32;
#   steeringTorqueDEPRECATED @3 :Float32;

#   actuators @6 :Actuators;

#   cruiseControl @4 :CruiseControl;
#   hudControl @5 :HUDControl;

#   struct Actuators {
#     # range from 0.0 - 1.0
#     gas @0: Float32;
#     brake @1: Float32;
#     # range from -1.0 - 1.0
#     steer @2: Float32;
#     steerAngle @3: Float32;
#   }

#   struct CruiseControl {
#     cancel @0: Bool;
#     override @1: Bool;
#     speedOverride @2: Float32;
#     accelOverride @3: Float32;
#   }

#   struct HUDControl {
#     speedVisible @0: Bool;
#     setSpeed @1: Float32;
#     lanesVisible @2: Bool;
#     leadVisible @3: Bool;
#     visualAlert @4: VisualAlert;
#     audibleAlert @5: AudibleAlert;
#     rightLaneVisible @6: Bool;
#     leftLaneVisible @7: Bool;
#     rightLaneDepart @8: Bool;
#     leftLaneDepart @9: Bool;

#     enum VisualAlert {
#       # these are the choices from the Honda
#       # map as good as you can for your car
#       none @0;
#       fcw @1;
#       steerRequired @2;
#       brakePressed @3;
#       wrongGear @4;
#       seatbeltUnbuckled @5;
#       speedTooHigh @6;
#       ldw @7;
#     }

#     enum AudibleAlert {
#       # these are the choices from the Honda
#       # map as good as you can for your car
#       none @0;
#       chimeEngage @1;
#       chimeDisengage @2;
#       chimeError @3;
#       chimeWarning1 @4;
#       chimeWarning2 @5;
#       chimeWarningRepeat @6;
#       chimePrompt @7;
#       chimeWarning2Repeat @8;
#     }
#   }
# }
