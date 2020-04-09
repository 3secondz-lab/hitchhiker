# ACRaw Msg

# From Assetto Corsa
std_msgs/Header header
std_msgs/String identifier
std_msgs/Int32 size
std_msgs/Float32 speed_Kmh
std_msgs/Float32 speed_Mph
std_msgs/Float32 speed_Ms
std_msgs/Bool isAbsEnabled
std_msgs/Bool isAbsInAction
std_msgs/Bool isTcInAction
std_msgs/Bool isTcEnabled
std_msgs/Bool isInPit
std_msgs/Bool isEngineLimiterOn
std_msgs/Float32 accG_vertical
std_msgs/Float32 accG_horizontal
std_msgs/Float32 accG_frontal
std_msgs/Int32 lapTime
std_msgs/Int32 lastLap
std_msgs/Int32 bestLap
std_msgs/Int32 lapCount
std_msgs/Float32 gas
std_msgs/Float32 brake
std_msgs/Float32 clutch
std_msgs/Float32 engineRPM
std_msgs/Float32 steer
std_msgs/Int32 gear
std_msgs/Float32 cgHeight
std_msgs/Float32MultiArray wheelAngularSpeed
std_msgs/Float32MultiArray slipAngle
std_msgs/Float32MultiArray slipAngle_ContactPatch
std_msgs/Float32MultiArray slipRatio
std_msgs/Float32MultiArray tyreSlip
std_msgs/Float32MultiArray ndSlip
std_msgs/Float32MultiArray load
std_msgs/Float32MultiArray Dy
std_msgs/Float32MultiArray Mz
std_msgs/Float32MultiArray tyreDirtyLevel
std_msgs/Float32MultiArray camberRAD
std_msgs/Float32MultiArray tyreRadius
std_msgs/Float32MultiArray tyreLoadedRadius
std_msgs/Float32MultiArray suspensionHeight
std_msgs/Float32 carPositionNormalized
std_msgs/Float32 carSlope
std_msgs/Float32MultiArray carCoordinates

# From parseai
# plane = [center_x, center_y, left_x, left_y, right_x, right_y, radius, psie, cte]
std_msgs/Float64MultiArray plane_center_x
std_msgs/Float64MultiArray plane_center_y
std_msgs/Float64MultiArray plane_left_x
std_msgs/Float64MultiArray plane_left_y
std_msgs/Float64MultiArray plane_right_x
std_msgs/Float64MultiArray plane_right_y
std_msgs/Float64MultiArray plane_radius
std_msgs/Float64 plane_psie
std_msgs/Float64 plane_cte

# cam = [center_x, center_y, left_x, left_y, right_x, right_y]
std_msgs/Float64MultiArray cam_center_x
std_msgs/Float64MultiArray cam_center_y
std_msgs/Float64MultiArray cam_left_x
std_msgs/Float64MultiArray cam_left_y
std_msgs/Float64MultiArray cam_right_x
std_msgs/Float64MultiArray cam_right_y
