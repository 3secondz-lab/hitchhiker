# HUDControl Message

std_msgs/Bool speedVisible
std_msgs/Float32 setSpeed
std_msgs/Bool lanesVisible
std_msgs/Bool leadVisible
std_msgs/String visualAlert
std_msgs/String audibleAlert
std_msgs/Bool rightLaneVisible
std_msgs/Bool leftLaneVisible
std_msgs/Bool rightLaneDepart
std_msgs/Bool leftLaneDepart



#  struct HUDControl {
#    speedVisible @0: Bool;
#    setSpeed @1: Float32;
#    lanesVisible @2: Bool;
#    leadVisible @3: Bool;
#    visualAlert @4: VisualAlert;
#    audibleAlert @5: AudibleAlert;
#    rightLaneVisible @6: Bool;
#    leftLaneVisible @7: Bool;
#    rightLaneDepart @8: Bool;
#    leftLaneDepart @9: Bool;
#
#    enum VisualAlert {
#      # these are the choices from the Honda
#      # map as good as you can for your car
#      none @0;
#      fcw @1;
#      steerRequired @2;
#      brakePressed @3;
#      wrongGear @4;
#      seatbeltUnbuckled @5;
#      speedTooHigh @6;
#      ldw @7;
#    }
#
#    enum AudibleAlert {
#      # these are the choices from the Honda
#      # map as good as you can for your car
#      none @0;
#      chimeEngage @1;
#      chimeDisengage @2;
#      chimeError @3;
#      chimeWarning1 @4;
#      chimeWarning2 @5;
#      chimeWarningRepeat @6;
#      chimePrompt @7;
#      chimeWarning2Repeat @8;
#    }
#  }
