Openpilot - ROS message converter
==========

## Data type conversion rules

| Cereal | ROS |
|:--------:|:-----:|
| Data | hex() -> std_msgs/String |
| Enum | std_msgs/String |
| xxxDEPRECATED | (Ignored) |
| (Duplicated name) | 1* |

1. There are some duplicated names in different structures. They are renamed with their superclass name (ex. LeadData in ModelData and RadarState -> LeadDataModel and LeadDataRadar).

## Notes

1. Openpilot should use zmq for communication. However, if we set ZMQ=1, settings panel does not work (no idea).
