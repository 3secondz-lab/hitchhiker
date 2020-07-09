# DrSvPolyReport Message

std_msgs/UInt16 svId
std_msgs/Int8 frequencyIndex

std_msgs/Bool hasPosition
std_msgs/Bool hasIono
std_msgs/Bool hasTropo
std_msgs/Bool hasElevation
std_msgs/Bool polyFromXtra
std_msgs/Bool hasSbasIono

std_msgs/UInt16 iode
std_msgs/Float64 t0
std_msgs/Float64[] xyz0
std_msgs/Float64[] xyzN
std_msgs/Float32[] other

std_msgs/Float32 positionUncertainty
std_msgs/Float32 ionoDelay
std_msgs/Float32 ionoDot
std_msgs/Float32 sbasIonoDelay
std_msgs/Float32 sbasIonoDot
std_msgs/Float32 tropoDelay
std_msgs/Float32 elevation
std_msgs/Float32 elevationDot
std_msgs/Float32 elevationUncertainty

std_msgs/Float64[] velocityCoeff
