# WifiScan Message

std_msgs/Header header

std_msgs/String bssid
std_msgs/String ssid
std_msgs/String capabilities
std_msgs/Int32 frequency
std_msgs/Int32 level
std_msgs/Int64 timestamp

std_msgs/Int32 centerFreq0
std_msgs/Int32 centerFreq1
std_msgs/UInt64 channelWidth
std_msgs/String operatorFriendlyName
std_msgs/String venueName
std_msgs/Bool is80211mcResponder
std_msgs/Bool passpoint

std_msgs/Int32 distanceCm
std_msgs/Int32 distanceSdCm



# struct WifiScan {
#   bssid @0 :Text;
#   ssid @1 :Text;
#   capabilities @2 :Text;
#   frequency @3 :Int32;
#   level @4 :Int32;
#   timestamp @5 :Int64;

#   centerFreq0 @6 :Int32;
#   centerFreq1 @7 :Int32;
#   channelWidth @8 :ChannelWidth;
#   operatorFriendlyName @9 :Text;
#   venueName @10 :Text;
#   is80211mcResponder @11 :Bool;
#   passpoint @12 :Bool;

#   distanceCm @13 :Int32;
#   distanceSdCm @14 :Int32;

#   enum ChannelWidth {
#     w20Mhz @0;
#     w40Mhz @1;
#     w80Mhz @2;
#     w160Mhz @3;
#     w80Plus80Mhz @4;
#   }
# }
