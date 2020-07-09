# NavStatus Message

std_msgs/Header header

std_msgs/Bool isNavigating
openpilot/Address currentAddress



# struct NavStatus {
#   isNavigating @0 :Bool;
#   currentAddress @1 :Address;

#   struct Address {
#     title @0 :Text;
#     lat @1 :Float64;
#     lng @2 :Float64;
#     house @3 :Text;
#     address @4 :Text;
#     street @5 :Text;
#     city @6 :Text;
#     state @7 :Text;
#     country @8 :Text;
#   }
# }
