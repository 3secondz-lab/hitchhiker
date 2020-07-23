# NavigationMessage Message

std_msgs/Int32 type
std_msgs/Int32 svId
std_msgs/Int32 messageId
std_msgs/Int32 submessageId
std_msgs/String data
std_msgs/String status



#  struct NavigationMessage {
#    type @0 :Int32;
#    svId @1 :Int32;
#    messageId @2 :Int32;
#    submessageId @3 :Int32;
#    data @4 :Data;
#    status @5 :Status;
#
#    enum Status {
#      unknown @0;
#      parityPassed @1;
#      parityRebuilt @2;
#    }
#  }
