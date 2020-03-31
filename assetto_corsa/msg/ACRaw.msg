# ACRaw Msg

# From Assetto Corsa
Header header
std_msgs/String identifier
int32 size
float32 speed_Kmh
float32 speed_Mph
float32 speed_Ms
bool isAbsEnabled
bool isAbsInAction
bool isTcInAction
bool isTcEnabled
bool isInPit
bool isEngineLimiterOn
float32 accG_vertical
float32 accG_horizontal
float32 accG_frontal
int32 lapTime
int32 lastLap
int32 bestLap
int32 lapCount
float32 gas
float32 brake
float32 clutch
float32 engineRPM
float32 steer
int32 gear
float32 cgHeight
float32[4] wheelAngularSpeed
float32[4] slipAngle
float32[4] slipAngle_ContactPatch
float32[4] slipRatio
float32[4] tyreSlip
float32[4] ndSlip
float32[4] load
float32[4] Dy
float32[4] Mz
float32[4] tyreDirtyLevel
float32[4] camberRAD
float32[4] tyreRadius
float32[4] tyreLoadedRadius
float32[4] suspensionHeight
float32 carPositionNormalized
float32 carSlope
float32[3] carCoordinates

# From parseai
# plane = [center_x, center_y, left_x, left_y, right_x, right_y, radius, psie, cte]
float64[] plane_center_x
float64[] plane_center_y
float64[] plane_left_x
float64[] plane_left_y
float64[] plane_right_x
float64[] plane_right_y
float64[] plane_radius
float64 plane_psie
float64 plane_cte

# cam = [center_x, center_y, left_x, left_y, right_x, right_y]
float64[] cam_center_x
float64[] cam_center_y
float64[] cam_left_x
float64[] cam_left_y
float64[] cam_right_x
float64[] cam_right_y