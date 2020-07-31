#!/usr/bin/python3
"""
    Openpilot-ROS message converter

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


# System
import os
os.environ["ZMQ"] = "1"

# Python
import numpy as np
import random
import string
import time
import datetime

# ROS
import rospy
from std_msgs import msg as sm
from openpilot import msg as om

# Openpilot
import cereal.messaging as messaging
from cereal import log, car
from capnp.lib import capnp
import zmq


class MessageConverter:
    dict_type = {
        # 'initData': om.InitData,
        'frame': om.FrameData,
        'gpsNMEA': om.GPSNMEAData,
        'can': om.CanDataList,
        'can_element': om.CanData,
        'thermal': om.ThermalData,
        'controlsState': om.ControlsState,
        'model': om.ModelData,
        'features': om.CalibrationFeatures,
        'sensorEvents': om.SensorEventDataList,
        'sensorEvents_element': om.SensorEventData,
        'health': om.HealthData,
        'radarState': om.RadarState,
        'encodeIdx': om.EncodeIndex,
        'liveTracks': om.LiveTracksList,
        'liveTracks_element': om.LiveTracks,
        'sendcan': om.SendCanDataList,
        'sendcan_element': om.CanData,
        'logMessage': om.LogMessage,
        'liveCalibration': om.LiveCalibrationData,
        'androidLog': om.AndroidLogEntry,
        'gpsLocation': om.GpsLocationData,
        'carState': om.CarState,
        'carControl': om.CarControl,
        'plan': om.Plan,
        'liveLocation': om.LiveLocationData,
        'ethernetData': om.EthernetPacketList,
        'ethernetData_element': om.EthernetPacket,
        'navUpdate': om.NavUpdate,
        # 'cellInfo': om.CellInfoList,
        # 'cellInfo_element': om.CellInfo,
        # 'wifiScan': om.WifiScanList,
        # 'wifiScan_element': om.WifiScan,
        # 'androidGnss': om.AndroidGnss,
        'qcomGnss': om.QcomGnss,
        'lidarPts': om.LidarPts,
        'procLog': om.ProcLog,
        'ubloxGnss': om.UbloxGnss,
        'clocks': om.Clocks,
        'liveMpc': om.LiveMpcData,
        'liveLongitudinalMpc': om.LiveLongitudinalMpcData,
        'navStatus': om.NavStatus,
        'ubloxRaw': om.UbloxRaw,
        'gpsPlannerPoints': om.GPSPlannerPoints,
        'gpsPlannerPlan': om.GPSPlannerPlan,
        'applanixRaw': om.ApplanixRaw,
        'trafficEvents': om.TrafficEventList,
        'trafficEvents_element': om.TrafficEvent,
        'liveLocationTiming': om.LiveLocationData,
        'liveLocationCorrected': om.LiveLocationData,
        'orbObservation': om.OrbObservationList,
        'orbObservation_element': om.OrbObservation,
        'gpsLocationExternal': om.GpsLocationData,
        # 'location': om.LiveLocationData,
        'uiNavigationEvent': om.UiNavigationEvent,
        'testJoystick': om.Joystick,
        'orbOdometry': om.OrbOdometry,
        'orbFeatures': om.OrbFeatures,
        'applanixLocation': om.LiveLocationData,
        'orbKeyFrame': om.OrbKeyFrame,
        'uiLayoutState': om.UiLayoutState,
        'orbFeaturesSummary': om.OrbFeaturesSummary,
        'driverState': om.DriverState,
        # 'boot': om.Boot,
        'liveParameters': om.LiveParametersData,
        'liveMapData': om.LiveMapData,
        'cameraOdometry': om.CameraOdometry,
        'pathPlan': om.PathPlan,
        'kalmanOdometry': om.KalmanOdometry,
        'thumbnail': om.Thumbnail,
        'carEvents': om.CarEventList,
        'carEvents_element': om.CarEvent,
        'carParams': om.CarParams,
        'frontFrame': om.FrameData,
        'dMonitoringState': om.DMonitoringState,
        'liveLocationKalman': om.LiveLocationKalman,
        # 'sentinel': om.Sentinel
    }

    def __init__(self, list_service):
        """
        Convert Openpilot services to ROS messages

        :param list_service: List of Openpilot services to subscribe
        """

        # struct Event {
        #     # in nanoseconds?
        #     logMonoTime @0 :UInt64;
        #     valid @67 :Bool = true;
        #
        #     union {
        #         initData @1 :InitData;
        #         frame @2 :FrameData;
        #         gpsNMEA @3 :GPSNMEAData;
        #         sensorEventDEPRECATED @4 :SensorEventData;
        #         can @5 :List(CanData);
        #         thermal @6 :ThermalData;
        #         controlsState @7 :ControlsState;
        #         liveEventDEPRECATED @8 :List(LiveEventData);
        #         model @9 :ModelData;
        #         features @10 :CalibrationFeatures;
        #         sensorEvents @11 :List(SensorEventData);
        #         health @12 :HealthData;
        #         radarState @13 :RadarState;
        #         liveUIDEPRECATED @14 :LiveUI;
        #         encodeIdx @15 :EncodeIndex;
        #         liveTracks @16 :List(LiveTracks);
        #         sendcan @17 :List(CanData);
        #         logMessage @18 :Text;
        #         liveCalibration @19 :LiveCalibrationData;
        #         androidLog @20 :AndroidLogEntry;
        #         gpsLocation @21 :GpsLocationData;
        #         carState @22 :Car.CarState;
        #         carControl @23 :Car.CarControl;
        #         plan @24 :Plan;
        #         liveLocation @25 :LiveLocationData;
        #         ethernetData @26 :List(EthernetPacket);
        #         navUpdate @27 :NavUpdate;
        #         cellInfo @28 :List(CellInfo);
        #         wifiScan @29 :List(WifiScan);
        #         androidGnss @30 :AndroidGnss;
        #         qcomGnss @31 :QcomGnss;
        #         lidarPts @32 :LidarPts;
        #         procLog @33 :ProcLog;
        #         ubloxGnss @34 :UbloxGnss;
        #         clocks @35 :Clocks;
        #         liveMpc @36 :LiveMpcData;
        #         liveLongitudinalMpc @37 :LiveLongitudinalMpcData;
        #         navStatus @38 :NavStatus;
        #         ubloxRaw @39 :Data;
        #         gpsPlannerPoints @40 :GPSPlannerPoints;
        #         gpsPlannerPlan @41 :GPSPlannerPlan;
        #         applanixRaw @42 :Data;
        #         trafficEvents @43 :List(TrafficEvent);
        #         liveLocationTiming @44 :LiveLocationData;
        #         orbslamCorrectionDEPRECATED @45 :OrbslamCorrection;
        #         liveLocationCorrected @46 :LiveLocationData;
        #         orbObservation @47 :List(OrbObservation);
        #         gpsLocationExternal @48 :GpsLocationData;
        #         location @49 :LiveLocationData;
        #         uiNavigationEvent @50 :UiNavigationEvent;
        #         liveLocationKalmanDEPRECATED @51 :LiveLocationData;
        #         testJoystick @52 :Joystick;
        #         orbOdometry @53 :OrbOdometry;
        #         orbFeatures @54 :OrbFeatures;
        #         applanixLocation @55 :LiveLocationData;
        #         orbKeyFrame @56 :OrbKeyFrame;
        #         uiLayoutState @57 :UiLayoutState;
        #         orbFeaturesSummary @58 :OrbFeaturesSummary;
        #         driverState @59 :DriverState;
        #         boot @60 :Boot;
        #         liveParameters @61 :LiveParametersData;
        #         liveMapData @62 :LiveMapData;
        #         cameraOdometry @63 :CameraOdometry;
        #         pathPlan @64 :PathPlan;
        #         kalmanOdometry @65 :KalmanOdometry;
        #         thumbnail @66: Thumbnail;
        #         carEvents @68: List(Car.CarEvent);
        #         carParams @69: Car.CarParams;
        #         frontFrame @70: FrameData;
        #         dMonitoringState @71: DMonitoringState;
        #         liveLocationKalman @72 :LiveLocationKalman;
        #         sentinel @73 :Sentinel;
        #     }
        # }

        self.list_service = list_service

        self.dict_pub = {}
        for s in self.list_service:
            self.dict_pub[s] = rospy.Publisher('~' + s, self.dict_type[s], queue_size=1)

        for k, v in self.dict_pub.items():
            print('Name : {}, Type : {}'.format(k, v.type))

    def publish(self, submaster):
        # Assume submaster is updated
        for s in self.list_service:
            try:
                if submaster.updated[s] and submaster.valid[s]:
                    # print(s)
                    msg = self.convert(s, submaster)
                    # msg = self.type_and_converter[s][1](submaster)

                    self.dict_pub[s].publish(msg)
            except KeyError:
                rospy.loginfo('Key {} is not available'.format(s))

    def convert(self, name, submaster):
        m = self.dict_type[name]()

        m.header.seq = submaster.frame
        m.header.stamp = rospy.Time.from_sec(submaster.logMonoTime[name] / 1e9)

        d = submaster.data[name]

        if type(d) == capnp._DynamicStructReader:
            m = self.convert_dict(m, d.to_dict())
        elif type(d) == capnp._DynamicListReader:
            list_d = list(d)
            att = getattr(m, name)
            for v in list_d:
                m_tmp = self.dict_type[name + '_element']()
                m_tmp = self.convert_dict(m_tmp, v.to_dict())
                att.append(m_tmp)
        elif type(d) == str:
            # LogMessage
            att = getattr(m, name)
            setattr(att, 'data', d)
        elif type(d) == bytes:
            # UbloxRaw
            att = getattr(m, name)
            setattr(att, 'data', d.decode())
        else:
            print('Unsupported type : {}'.format(type(d)))

        return m

    def convert_dict(self, msg, dict_data):
        for k, v in dict_data.items():
            # print('{} : {}'.format(k, v))
            if 'deprecated' in k.lower():
                continue

            # Special treatment for from
            if k == 'from':
                k = 'from_'

            if type(v) == list:
                att = getattr(msg, k)
                att_type = msg._slot_types[msg.__slots__.index(k)]

                if att_type[-2:] != '[]':
                    print('Message {} / {} : List attribute type is expected but got {}'.format(type(msg), k, att_type))
                    continue
                else:
                    att_type = att_type[:-2]

                if 'std_msgs/' in att_type:
                    att_type = att_type.replace('std_msgs/', '')
                    m_class = getattr(sm, att_type)

                    v_conv = []
                    for val in v:
                        v_conv_m = m_class()
                        setattr(v_conv_m, 'data', val)
                        v_conv.append(v_conv_m)

                    setattr(msg, k, v_conv)
                elif 'openpilot/' in att_type:
                    att_type = att_type.replace('openpilot/', '')
                    m_class = getattr(om, att_type)

                    v_conv = []
                    for val in v:
                        v_conv_m = m_class()
                        v_conv_m = self.convert_dict(v_conv_m, val)
                        v_conv.append(v_conv_m)

                    setattr(msg, k, v_conv)
                else:
                    print('Message {} / {} : Unrecognized type {}'.format(type(msg), k, att_type))
                    continue
            elif type(v) == dict:
                if hasattr(msg, k):
                    att = getattr(msg, k)
                    self.convert_dict(att, v)
                else:
                    for k2, v2 in v.items():
                        att = getattr(msg, k2)
                        self.convert_dict(att, v2)
            elif type(v) == bytes:
                att = getattr(msg, k)
                setattr(att, 'data', v.decode())
            else:
                att = getattr(msg, k)
                setattr(att, 'data', v)

        return msg


def test():
    # For test
    rospy.init_node('msg_conv_test')

    dict_data = {}

    # # frame
    # dict_data['frame'] = messaging.new_message('frame')
    # dict_data['frame'].frame.image = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['frame'].frame.focusVal = [1, 2, 3, -4, -5]
    # dict_data['frame'].frame.focusConf = [0, 128, 255]
    # dict_data['frame'].frame.sharpnessScore = [0, 65535, 500]
    # dict_data['frame'].frame.frameType = 'neo'
    # dict_data['frame'].frame.transform = [-31.56, 12321.954]
    # dict_data['frame'].frame.androidCaptureResult.colorCorrectionTransform = [4, -7, 9]
    # dict_data['frame'].frame.androidCaptureResult.colorCorrectionGains = [4.0, 79.0, -53.4]

    # # gpsNMEA
    # dict_data['gpsNMEA'] = messaging.new_message('gpsNMEA')
    # dict_data['gpsNMEA'].gpsNMEA.nmea = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # can
    # dict_data['can'] = messaging.new_message('can', size=2)
    # dict_data['can'].can[0].dat = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['can'].can[1].dat = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # thermal
    # dict_data['thermal'] = messaging.new_message('thermal')
    # dict_data['thermal'].thermal.batteryStatus = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['thermal'].thermal.networkType = 'cell5G'
    # dict_data['thermal'].thermal.networkStrength = 'good'
    # dict_data['thermal'].thermal.thermalStatus = 'yellow'

    # # controlsState
    # dict_data['controlsState'] = messaging.new_message('controlsState')
    # dict_data['controlsState'].controlsState.canMonoTimes = [231423, 452625, 567547]
    # dict_data['controlsState'].controlsState.state = 'enabled'
    # dict_data['controlsState'].controlsState.longControlState = 'starting'
    # dict_data['controlsState'].controlsState.alertText1 = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['controlsState'].controlsState.alertText2 = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['controlsState'].controlsState.alertStatus = 'normal'
    # dict_data['controlsState'].controlsState.alertSize = 'mid'
    # dict_data['controlsState'].controlsState.alertType = 'tmp'
    # dict_data['controlsState'].controlsState.alertSound = 'chimePrompt'
    # dict_data['controlsState'].controlsState.lateralControlState.pidState = log.ControlsState.LateralPIDState.new_message()
    # dict_data['controlsState'].controlsState.lateralControlState.pidState.output = 846.546

    # # model
    # dict_data['model'] = messaging.new_message('model')
    # dict_data['model'].model.path.points = [-13.433, 343.0087, 437932.2934]
    # dict_data['model'].model.path.stds = [-13452.433, 34423.008, 432.2307]
    # dict_data['model'].model.path.poly = [-45.3, 78.87, 96.2174]
    # dict_data['model'].model.leftLane.points = [-1879.433897, 379843.0654087, 4356847932.24685934]
    # dict_data['model'].model.leftLane.stds = [-8.689, 89467.87946, 5143.54]
    # dict_data['model'].model.leftLane.poly = [5.5684, -546.5, 486]
    # dict_data['model'].model.rightLane.points = [4568.5648, 4687.456, -8.808796]
    # dict_data['model'].model.rightLane.stds = [-534.87, 4253.867, 2543.243]
    # dict_data['model'].model.rightLane.poly = [978.64, 243.563, -54.532]
    # dict_data['model'].model.lead.relAStd = 876.98
    # dict_data['model'].model.freePath = [234.43, 567.876, -54.5]
    # dict_data['model'].model.settings.boxProjection = [345.65, 23.65]
    # dict_data['model'].model.settings.yuvCorrection = [32.8, 438.9]
    # dict_data['model'].model.settings.inputTransform = [9.0, 8.67]
    # dict_data['model'].model.leadFuture.relAStd = 445.578
    # dict_data['model'].model.speed = [34.768, 243.657]
    # dict_data['model'].model.meta.desirePrediction = [45.5674, 34.7]
    # dict_data['model'].model.meta.desireState = [23.65, 23.87]
    # dict_data['model'].model.longitudinal.distances = [43.675, 243.567]
    # dict_data['model'].model.longitudinal.speeds = [4.6, 2435.76]
    # dict_data['model'].model.longitudinal.accelerations = [43.6, 324.8]

    # # features
    # dict_data['features'] = messaging.new_message('features')
    # dict_data['features'].features.p0 = [3214.32, 213.1]
    # dict_data['features'].features.p1 = [34.3, 2.1324]
    # dict_data['features'].features.status = [-7, 5, 3]

    # # sensorEvents
    # dict_data['sensorEvents'] = messaging.new_message('sensorEvents', size=2)
    # dict_data['sensorEvents'].sensorEvents[0].acceleration.v = [14.1, -324.1]
    # dict_data['sensorEvents'].sensorEvents[0].acceleration.status = 7
    # dict_data['sensorEvents'].sensorEvents[0].source = 'bmp280'
    # dict_data['sensorEvents'].sensorEvents[1].proximity = 234.1
    # dict_data['sensorEvents'].sensorEvents[1].source = 'velodyne'

    # # health
    # dict_data['health'] = messaging.new_message('health')
    # dict_data['health'].health.voltage = 20000
    # dict_data['health'].health.usbPowerMode = 'cdp'
    # dict_data['health'].health.hwType = 'greyPanda'
    # dict_data['health'].health.faultStatus = 'faultTemp'
    # dict_data['health'].health.faults = ['interruptRateGmlan', 'interruptRateUsb']

    # # radarState
    # dict_data['radarState'] = messaging.new_message('radarState')
    # dict_data['radarState'].radarState.canMonoTimes = [121, 45261125, 997786721]
    # dict_data['radarState'].radarState.mdMonoTime = 59876986
    # dict_data['radarState'].radarState.controlsStateMonoTime = 9878546243132
    # dict_data['radarState'].radarState.radarErrors = ['canError', 'fault', 'wrongConfig']
    # dict_data['radarState'].radarState.leadOne.dPath = -5467.06867
    # dict_data['radarState'].radarState.leadOne.radar = True
    # dict_data['radarState'].radarState.leadTwo.vRel = 78971.09874
    # dict_data['radarState'].radarState.leadTwo.fcw = True

    # # encodeIdx
    # dict_data['encodeIdx'] = messaging.new_message('encodeIdx')
    # dict_data['encodeIdx'].encodeIdx.type = 'bigBoxHEVC'
    # dict_data['encodeIdx'].encodeIdx.segmentIdEncode = 5687

    # # liveTracks
    # dict_data['liveTracks'] = messaging.new_message('liveTracks', size=2)
    # dict_data['liveTracks'].liveTracks[0].stationary = True
    # dict_data['liveTracks'].liveTracks[1].aRel = -4563.478

    # # sendcan
    # dict_data['sendcan'] = messaging.new_message('sendcan', size=2)
    # dict_data['sendcan'].sendcan[0].dat = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['sendcan'].sendcan[1].dat = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # logMessage
    # dict_data['logMessage'] = messaging.new_message('logMessage', size=10)
    # dict_data['logMessage'].logMessage = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # liveCalibration
    # dict_data['liveCalibration'] = messaging.new_message('liveCalibration')
    # dict_data['liveCalibration'].liveCalibration.warpMatrix = [-234.1234, 134550.0342, 2139.1]
    # dict_data['liveCalibration'].liveCalibration.warpMatrix2 = [23423, 45254, 454.43, 134404.55, 234.0]
    # dict_data['liveCalibration'].liveCalibration.warpMatrixBig = [-4567.879, 15696.443]
    # dict_data['liveCalibration'].liveCalibration.extrinsicMatrix = [-3242.03, -1.34235, -1234.1, 33.2]
    # dict_data['liveCalibration'].liveCalibration.rpyCalib = [12412.121, 21341.0, -32423.0]

    # # androidLog
    # dict_data['androidLog'] = messaging.new_message('androidLog')
    # dict_data['androidLog'].androidLog.ts = 24352456908
    # dict_data['androidLog'].androidLog.tag = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['androidLog'].androidLog.message = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # gpsLocation
    # dict_data['gpsLocation'] = messaging.new_message('gpsLocation')
    # dict_data['gpsLocation'].gpsLocation.longitude = 127.024612
    # dict_data['gpsLocation'].gpsLocation.latitude = 37.532600
    # dict_data['gpsLocation'].gpsLocation.source = 'fusion'
    # dict_data['gpsLocation'].gpsLocation.vNED = [1.3242, 132.436, 321.5098]

    # # carState
    # dict_data['carState'] = messaging.new_message('carState')
    # dict_data['carState'].carState.events = [car.CarEvent.new_message(), car.CarEvent.new_message()]
    # dict_data['carState'].carState.events[0].name = 'brakeUnavailable'
    # dict_data['carState'].carState.events[0].permanent = True
    # dict_data['carState'].carState.events[1].name = 'preDriverDistracted'
    # dict_data['carState'].carState.events[1].userDisable = True
    # dict_data['carState'].carState.wheelSpeeds.fl = 12.59
    # dict_data['carState'].carState.cruiseState.enabled = True
    # dict_data['carState'].carState.gearShifter = 'neutral'
    # dict_data['carState'].carState.buttonEvents = [car.CarState.ButtonEvent.new_message(), car.CarState.ButtonEvent.new_message()]
    # dict_data['carState'].carState.buttonEvents[0].pressed = True
    # dict_data['carState'].carState.buttonEvents[0].type = 'leftBlinker'
    # dict_data['carState'].carState.buttonEvents[1].pressed = True
    # dict_data['carState'].carState.buttonEvents[1].type = 'resumeCruise'
    # dict_data['carState'].carState.canMonoTimes = [121, 45261125, 997786721]

    # # carControl
    # dict_data['carControl'] = messaging.new_message('carControl')
    # dict_data['carControl'].carControl.actuators.steerAngle = 10.25
    # dict_data['carControl'].carControl.cruiseControl.override = True
    # dict_data['carControl'].carControl.cruiseControl.accelOverride = -1.2456
    # dict_data['carControl'].carControl.hudControl.visualAlert = 'wrongGear'
    # dict_data['carControl'].carControl.hudControl.audibleAlert = 'chimeWarningRepeat'

    # # plan
    # dict_data['plan'] = messaging.new_message('plan')
    # dict_data['plan'].plan.longitudinalPlanSource = 'mpc3'
    # dict_data['plan'].plan.gpsTrajectory.x = [123.1, 123.5, 435.5]
    # dict_data['plan'].plan.gpsTrajectory.y = [456.8, 6587.98, -123.0]
    # dict_data['plan'].plan.radarCanError = True

    # # liveLocation
    # dict_data['liveLocation'] = messaging.new_message('liveLocation')
    # dict_data['liveLocation'].liveLocation.vNED = [1232.54, 1341.56, 540.4]
    # dict_data['liveLocation'].liveLocation.gyro = [13232.54, 156341.566, 5430.4547]
    # dict_data['liveLocation'].liveLocation.accel = [-435.5, 345.0, 433.021]
    # dict_data['liveLocation'].liveLocation.accuracy.pNEDError = [1231, 3467.0]
    # dict_data['liveLocation'].liveLocation.accuracy.vNEDError = [1232.3131, -534.099]
    # dict_data['liveLocation'].liveLocation.source = 'orbslam'
    # dict_data['liveLocation'].liveLocation.positionECEF = [123.6, 4923.21]
    # dict_data['liveLocation'].liveLocation.poseQuatECEF = [234.0, 34.0, 143.0, 232.09, 3453.0]
    # dict_data['liveLocation'].liveLocation.imuFrame = [34.0, 2372.029, 355453.0]

    # # ethernetData
    # dict_data['ethernetData'] = messaging.new_message('ethernetData', size=2)
    # dict_data['ethernetData'].ethernetData = [log.EthernetPacket.new_message(), log.EthernetPacket.new_message()]
    # dict_data['ethernetData'].ethernetData[0].pkt = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['ethernetData'].ethernetData[0].ts = 23451345143.134
    # dict_data['ethernetData'].ethernetData[1].pkt = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['ethernetData'].ethernetData[1].ts = 45565.8698798

    # # navUpdate
    # dict_data['navUpdate'] = messaging.new_message('navUpdate')
    # dict_data['navUpdate'].navUpdate.segments = [log.NavUpdate.Segment.new_message(), log.NavUpdate.Segment.new_message()]
    # att = getattr(dict_data['navUpdate'].navUpdate.segments[0], 'from')
    # setattr(att, 'lat', 3453.1)
    # setattr(att, 'lng', 785.0)
    # dict_data['navUpdate'].navUpdate.segments[0].instruction = 'unkn8'
    # dict_data['navUpdate'].navUpdate.segments[0].parts = [log.NavUpdate.LatLng.new_message(), log.NavUpdate.LatLng.new_message()]
    # dict_data['navUpdate'].navUpdate.segments[0].parts[0].lat = 1.0
    # dict_data['navUpdate'].navUpdate.segments[0].parts[1].lng = -1.0
    # att = getattr(dict_data['navUpdate'].navUpdate.segments[1], 'from')
    # setattr(att, 'lat', 5464.0)
    # setattr(att, 'lng', -4564.0)
    # dict_data['navUpdate'].navUpdate.segments[1].instruction = 'roundaboutTurnLeft'
    # dict_data['navUpdate'].navUpdate.segments[1].parts = [log.NavUpdate.LatLng.new_message(), log.NavUpdate.LatLng.new_message()]
    # dict_data['navUpdate'].navUpdate.segments[1].parts[0].lat = -41.0
    # dict_data['navUpdate'].navUpdate.segments[1].parts[1].lng = 5450.0

    # # qcomGnss
    # dict_data['qcomGnss'] = messaging.new_message('qcomGnss')
    #
    # dict_data['qcomGnss'].qcomGnss.measurementReport = log.QcomGnss.MeasurementReport.new_message()
    # dict_data['qcomGnss'].qcomGnss.measurementReport.source = 'beidou'
    # dict_data['qcomGnss'].qcomGnss.measurementReport.sv = [log.QcomGnss.MeasurementReport.SV.new_message(), log.QcomGnss.MeasurementReport.SV.new_message()]
    # dict_data['qcomGnss'].qcomGnss.measurementReport.sv[0].observationState = 'dpo'
    # dict_data['qcomGnss'].qcomGnss.measurementReport.sv[0].measurementStatus.lastUpdateFromVelocityDifference = True
    # dict_data['qcomGnss'].qcomGnss.measurementReport.sv[1].observationState = 'glo10msAt'
    # dict_data['qcomGnss'].qcomGnss.measurementReport.sv[1].measurementStatus.lockPointValid = True
    #
    # dict_data['qcomGnss'].qcomGnss.clockReport = log.QcomGnss.ClockReport.new_message()
    # dict_data['qcomGnss'].qcomGnss.clockReport.hasRtcTime = True
    #
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport = log.QcomGnss.DrMeasurementReport.new_message()
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.source = 'glonass'
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv = [log.QcomGnss.DrMeasurementReport.SV.new_message(), log.QcomGnss.DrMeasurementReport.SV.new_message()]
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv[0].observationState = 'trackVerify'
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv[0].measurementStatus.sirCheckIsNeeded = True
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv[1].observationState = 'restart'
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv[1].measurementStatus.multipathIndicator = True
    # dict_data['qcomGnss'].qcomGnss.drMeasurementReport.sv[1].goodParity = True
    #
    # dict_data['qcomGnss'].qcomGnss.drSvPoly = log.QcomGnss.DrSvPolyReport.new_message()
    # dict_data['qcomGnss'].qcomGnss.drSvPoly.xyz0 = [234123.1, 23141234.5, 456456.2]
    # dict_data['qcomGnss'].qcomGnss.drSvPoly.xyzN = [435.3, 12346.7, 878.5]
    # dict_data['qcomGnss'].qcomGnss.drSvPoly.other = [32.5, 3443.7, 6556.8]
    # dict_data['qcomGnss'].qcomGnss.drSvPoly.velocityCoeff = [123.5, 324523.76, 56456.3, 4536.8]
    #
    # dict_data['qcomGnss'].qcomGnss.rawLog = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # lidarPts
    # dict_data['lidarPts'] = messaging.new_message('lidarPts')
    # dict_data['lidarPts'].lidarPts.r = [123, 435, 1234]
    # dict_data['lidarPts'].lidarPts.theta = [332, 11, 15]
    # dict_data['lidarPts'].lidarPts.reflect = [0, 255, 53]
    # dict_data['lidarPts'].lidarPts.idx = 4566878
    # dict_data['lidarPts'].lidarPts.pkt = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # procLog
    # dict_data['procLog'] = messaging.new_message('procLog')
    # dict_data['procLog'].procLog.cpuTimes = [log.ProcLog.CPUTimes.new_message(), log.ProcLog.CPUTimes.new_message()]
    # dict_data['procLog'].procLog.cpuTimes[0].cpuNum = 34
    # dict_data['procLog'].procLog.cpuTimes[1].idle = 43543.2
    # dict_data['procLog'].procLog.mem.shared = 234123
    # dict_data['procLog'].procLog.procs = [log.ProcLog.Process.new_message(), log.ProcLog.Process.new_message()]
    # dict_data['procLog'].procLog.procs[0].name = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['procLog'].procLog.procs[0].cmdline = [''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10)), ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))]
    # dict_data['procLog'].procLog.procs[0].exe = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['procLog'].procLog.procs[0].numThreads = 24
    # dict_data['procLog'].procLog.procs[1].name = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['procLog'].procLog.procs[1].cmdline = [''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10)), ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))]
    # dict_data['procLog'].procLog.procs[1].exe = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['procLog'].procLog.procs[1].processor = 34523

    # # ubloxGnss
    # dict_data['ubloxGnss'] = messaging.new_message('ubloxGnss')
    #
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport = log.UbloxGnss.MeasurementReport.new_message()
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.receiverStatus.clkReset = True
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.measurements = [log.UbloxGnss.MeasurementReport.Measurement.new_message(), log.UbloxGnss.MeasurementReport.Measurement.new_message()]
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.measurements[0].trackingStatus.halfCycleSubtracted = True
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.measurements[0].locktime = 234
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.measurements[1].trackingStatus.carrierPhaseValid = True
    # dict_data['ubloxGnss'].ubloxGnss.measurementReport.measurements[1].dopplerStdev = 43534.3
    #
    # dict_data['ubloxGnss'].ubloxGnss.ephemeris = log.UbloxGnss.Ephemeris.new_message()
    # dict_data['ubloxGnss'].ubloxGnss.ephemeris.ionoAlpha = [2332.3, 3141.5, 1345143.5]
    # dict_data['ubloxGnss'].ubloxGnss.ephemeris.ionoBeta = [546.3, 34243.76, 45252.45]
    #
    # dict_data['ubloxGnss'].ubloxGnss.ionoData = log.UbloxGnss.IonoData.new_message()
    # dict_data['ubloxGnss'].ubloxGnss.ionoData.ionoAlpha = [5446.3, 3422343.76, 45253422.45]
    # dict_data['ubloxGnss'].ubloxGnss.ionoData.ionoBeta = [4564.4, 345.6, 43.65]
    #
    # dict_data['ubloxGnss'].ubloxGnss.hwStatus = log.UbloxGnss.HwStatus.new_message()
    # dict_data['ubloxGnss'].ubloxGnss.hwStatus.aStatus = 'ok'
    # dict_data['ubloxGnss'].ubloxGnss.hwStatus.aPower = 'dontknow'
    # dict_data['ubloxGnss'].ubloxGnss.hwStatus.noisePerMS = 344

    # # clocks
    # dict_data['clocks'] = messaging.new_message('clocks')
    # dict_data['clocks'].clocks.monotonicRawNanos = 23465897

    # # liveMpc
    # dict_data['liveMpc'] = messaging.new_message('liveMpc')
    # dict_data['liveMpc'].liveMpc.x = [213, 234.1, 1234.1]
    # dict_data['liveMpc'].liveMpc.y = [567.5, 453.2, -4325.1]
    # dict_data['liveMpc'].liveMpc.psi = [567.5, 453.2, -4325.1]
    # dict_data['liveMpc'].liveMpc.delta = [213, 234.1, 1234.1]

    # # liveLongitudinalMpc
    # dict_data['liveLongitudinalMpc'] = messaging.new_message('liveLongitudinalMpc')
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.xEgo = [8.1, 45.4]
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.vEgo = [5.1, 2.4]
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.aEgo = [7.1, 7.4]
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.xLead = [9.1, 0.4]
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.vLead = [3.1, 5.4]
    # dict_data['liveLongitudinalMpc'].liveLongitudinalMpc.aLead = [23.1, 4.4]

    # # navStatus
    # dict_data['navStatus'] = messaging.new_message('navStatus')
    # dict_data['navStatus'].navStatus.isNavigating = True
    # dict_data['navStatus'].navStatus.currentAddress.street = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # ubloxRaw
    # dict_data['ubloxRaw'] = messaging.new_message('ubloxRaw', size=10)
    # dict_data['ubloxRaw'].ubloxRaw = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # gpsPlannerPoints
    # dict_data['gpsPlannerPoints'] = messaging.new_message('gpsPlannerPoints')
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points = [log.ECEFPoint.new_message(), log.ECEFPoint.new_message()]
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[0].x = 3443.1
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[0].y = 124.1
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[0].z = 7876.1
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[1].x = 4352435.2
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[1].y = 56756.4
    # dict_data['gpsPlannerPoints'].gpsPlannerPoints.points[1].z = 3223.2

    # # gpsPlannerPlan
    # dict_data['gpsPlannerPlan'] = messaging.new_message('gpsPlannerPlan')
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.poly = [1.05, 43.87, -542.2]
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points = [log.ECEFPoint.new_message(), log.ECEFPoint.new_message()]
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[0].x = 3443.1
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[0].y = 124.1
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[0].z = 7876.1
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[1].x = 4352435.2
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[1].y = 56756.4
    # dict_data['gpsPlannerPlan'].gpsPlannerPlan.points[1].z = 3223.2

    # # applanixRaw
    # dict_data['applanixRaw'] = messaging.new_message('applanixRaw', size=20)
    # dict_data['applanixRaw'].applanixRaw = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # trafficEvents
    # dict_data['trafficEvents'] = messaging.new_message('trafficEvents', size=2)
    # dict_data['trafficEvents'].trafficEvents[0].type = 'lightGreen'
    # dict_data['trafficEvents'].trafficEvents[0].action = 'resumeReady'
    # dict_data['trafficEvents'].trafficEvents[1].type = 'lightYellow'
    # dict_data['trafficEvents'].trafficEvents[1].action = 'yield'

    # # liveLocationTiming
    # dict_data['liveLocationTiming'] = messaging.new_message('liveLocationTiming')
    # dict_data['liveLocationTiming'].liveLocationTiming.vNED = [1231.1, 23432.2, 21341234.5]
    # dict_data['liveLocationTiming'].liveLocationTiming.gyro = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationTiming'].liveLocationTiming.accel = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationTiming'].liveLocationTiming.accuracy.pNEDError = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationTiming'].liveLocationTiming.accuracy.vNEDError = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationTiming'].liveLocationTiming.source = 'timing'
    # dict_data['liveLocationTiming'].liveLocationTiming.positionECEF = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationTiming'].liveLocationTiming.poseQuatECEF = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationTiming'].liveLocationTiming.imuFrame = [45.1, 77.2, 8799.5]

    # # liveLocationCorrected
    # dict_data['liveLocationCorrected'] = messaging.new_message('liveLocationCorrected')
    # dict_data['liveLocationCorrected'].liveLocationCorrected.vNED = [1231.1, 23432.2, 21341234.5]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.gyro = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.accel = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.accuracy.pNEDError = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.accuracy.vNEDError = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.source = 'timing'
    # dict_data['liveLocationCorrected'].liveLocationCorrected.positionECEF = [3243.2, 3545.6, 34534.7]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.poseQuatECEF = [45.1, 77.2, 8799.5]
    # dict_data['liveLocationCorrected'].liveLocationCorrected.imuFrame = [45.1, 77.2, 8799.5]

    # # orbObservation
    # dict_data['orbObservation'] = messaging.new_message('orbObservation', size=2)
    # dict_data['orbObservation'].orbObservation[0].normalizedCoordinates = [123.4, 2345.2]
    # dict_data['orbObservation'].orbObservation[0].locationECEF = [21323.4, 2341435.2]
    # dict_data['orbObservation'].orbObservation[1].normalizedCoordinates = [125433.2434, 2345.215]
    # dict_data['orbObservation'].orbObservation[1].locationECEF = [2.464, 35.21345143]

    # # gpsLocationExternal
    # dict_data['gpsLocationExternal'] = messaging.new_message('gpsLocationExternal')
    # dict_data['gpsLocationExternal'].gpsLocationExternal.longitude = 127.024612
    # dict_data['gpsLocationExternal'].gpsLocationExternal.latitude = 37.532600
    # dict_data['gpsLocationExternal'].gpsLocationExternal.source = 'fusion'
    # dict_data['gpsLocationExternal'].gpsLocationExternal.vNED = [1.3242, 132.436, 321.5098]

    # # uiNavigationEvent
    # dict_data['uiNavigationEvent'] = messaging.new_message('uiNavigationEvent')
    # dict_data['uiNavigationEvent'].uiNavigationEvent.type = 'mergeRight'
    # dict_data['uiNavigationEvent'].uiNavigationEvent.status = 'approaching'
    # dict_data['uiNavigationEvent'].uiNavigationEvent.endRoadPoint.x = 322
    # dict_data['uiNavigationEvent'].uiNavigationEvent.endRoadPoint.y = 76.54
    # dict_data['uiNavigationEvent'].uiNavigationEvent.endRoadPoint.z = 8.56

    # # testJoystick
    # dict_data['testJoystick'] = messaging.new_message('testJoystick')
    # dict_data['testJoystick'].testJoystick.axes = [32423.1, 3434.2, 54645.2]
    # dict_data['testJoystick'].testJoystick.buttons = [True, False, True]

    # # orbOdometry
    # dict_data['orbOdometry'] = messaging.new_message('orbOdometry')
    # dict_data['orbOdometry'].orbOdometry.f = [3243.5, 4352.1, 345243.1]
    # dict_data['orbOdometry'].orbOdometry.matches = [425, 2624, -1]

    # # orbFeatures
    # dict_data['orbFeatures'] = messaging.new_message('orbFeatures')
    # dict_data['orbFeatures'].orbFeatures.xs = [324, 546254.26]
    # dict_data['orbFeatures'].orbFeatures.ys = [3234233.525, 546254.26]
    # dict_data['orbFeatures'].orbFeatures.descriptors = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['orbFeatures'].orbFeatures.octaves = [3, 4, 5]
    # dict_data['orbFeatures'].orbFeatures.matches = [12, 556, 2677]

    # # applanixLocation
    # dict_data['applanixLocation'] = messaging.new_message('applanixLocation')
    # dict_data['applanixLocation'].applanixLocation.vNED = [1231.1, 23432.2, 21341234.5]
    # dict_data['applanixLocation'].applanixLocation.gyro = [45.1, 77.2, 8799.5]
    # dict_data['applanixLocation'].applanixLocation.accel = [3243.2, 3545.6, 34534.7]
    # dict_data['applanixLocation'].applanixLocation.accuracy.pNEDError = [3243.2, 3545.6, 34534.7]
    # dict_data['applanixLocation'].applanixLocation.accuracy.vNEDError = [45.1, 77.2, 8799.5]
    # dict_data['applanixLocation'].applanixLocation.source = 'timing'
    # dict_data['applanixLocation'].applanixLocation.positionECEF = [3243.2, 3545.6, 34534.7]
    # dict_data['applanixLocation'].applanixLocation.poseQuatECEF = [45.1, 77.2, 8799.5]
    # dict_data['applanixLocation'].applanixLocation.imuFrame = [45.1, 77.2, 8799.5]

    # # orbKeyFrame
    # dict_data['orbKeyFrame'] = messaging.new_message('orbKeyFrame')
    # dict_data['orbKeyFrame'].orbKeyFrame.pos.x = 31.2
    # dict_data['orbKeyFrame'].orbKeyFrame.pos.y = 13316.24
    # dict_data['orbKeyFrame'].orbKeyFrame.pos.z = 317.342
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos = [log.ECEFPoint.new_message(), log.ECEFPoint.new_message()]
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[0].x = 3443.1
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[0].y = 124.1
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[0].z = 7876.1
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[1].x = 4352435.2
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[1].y = 56756.4
    # dict_data['orbKeyFrame'].orbKeyFrame.dpos[1].z = 3223.2
    # dict_data['orbKeyFrame'].orbKeyFrame.descriptors = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # uiLayoutState
    # dict_data['uiLayoutState'] = messaging.new_message('uiLayoutState')
    # dict_data['uiLayoutState'].uiLayoutState.activeApp = 'settings'
    # dict_data['uiLayoutState'].uiLayoutState.mapEnabled = True

    # # orbFeaturesSummary
    # dict_data['orbFeaturesSummary'] = messaging.new_message('orbFeaturesSummary')
    # dict_data['orbFeaturesSummary'].orbFeaturesSummary.matchCount = 3424

    # # driverState
    # dict_data['driverState'] = messaging.new_message('driverState')
    # dict_data['driverState'].driverState.faceOrientation = [45.1, 77.2, 8799.5]
    # dict_data['driverState'].driverState.facePosition = [234.1, 6.2, 8.5]
    # dict_data['driverState'].driverState.faceOrientationStd = [3425.1, 6.2, 8.805]
    # dict_data['driverState'].driverState.facePositionStd = [7.1, 6.2564, 8.97]

    # # liveParameters
    # dict_data['liveParameters'] = messaging.new_message('liveParameters')
    # dict_data['liveParameters'].liveParameters.sensorValid = True

    # # liveMapData
    # dict_data['liveMapData'] = messaging.new_message('liveMapData')
    # dict_data['liveMapData'].liveMapData.curvatureValid = True
    # dict_data['liveMapData'].liveMapData.lastGps.vNED = [453.1, 4252.6, 1345.1]
    # dict_data['liveMapData'].liveMapData.roadX = [3421.1, 435423.6]
    # dict_data['liveMapData'].liveMapData.roadY = [34421.1, 43574237.6]
    # dict_data['liveMapData'].liveMapData.roadCurvatureX = [3421.1, 435423.6]
    # dict_data['liveMapData'].liveMapData.roadCurvature = [34421.1, 43574237.6]

    # # cameraOdometry
    # dict_data['cameraOdometry'] = messaging.new_message('cameraOdometry')
    # dict_data['cameraOdometry'].cameraOdometry.trans = [4353, 677.65, 5788.43]
    # dict_data['cameraOdometry'].cameraOdometry.rot = [7, 98.65, 9.43]
    # dict_data['cameraOdometry'].cameraOdometry.transStd = [864, 0.65, 3.43]
    # dict_data['cameraOdometry'].cameraOdometry.rotStd = [33.21, 7.65, 67.43]

    # # pathPlan
    # dict_data['pathPlan'] = messaging.new_message('pathPlan')
    # dict_data['pathPlan'].pathPlan.dPoly = [864, 0.65, 3.43]
    # dict_data['pathPlan'].pathPlan.cPoly = [34, 0.655, 3.413]
    # dict_data['pathPlan'].pathPlan.lPoly = [2345234, 20.655, 3897.413]
    # dict_data['pathPlan'].pathPlan.rPoly = [3874, 0.356655, 3.41873]
    # dict_data['pathPlan'].pathPlan.desire = 'keepLeft'
    # dict_data['pathPlan'].pathPlan.laneChangeState = 'laneChangeStarting'
    # dict_data['pathPlan'].pathPlan.laneChangeDirection = 'left'

    # # kalmanOdometry
    # dict_data['kalmanOdometry'] = messaging.new_message('kalmanOdometry')
    # dict_data['kalmanOdometry'].kalmanOdometry.trans = [4353, 677.65, 5788.43]
    # dict_data['kalmanOdometry'].kalmanOdometry.rot = [7, 98.65, 9.43]
    # dict_data['kalmanOdometry'].kalmanOdometry.transStd = [864, 0.65, 3.43]
    # dict_data['kalmanOdometry'].kalmanOdometry.rotStd = [33.21, 7.65, 67.43]

    # # thumbnail
    # dict_data['thumbnail'] = messaging.new_message('thumbnail')
    # dict_data['thumbnail'].thumbnail.thumbnail = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))

    # # carEvents
    # dict_data['carEvents'] = messaging.new_message('carEvents', size=2)
    # dict_data['carEvents'].carEvents[0].name = 'brakeUnavailable'
    # dict_data['carEvents'].carEvents[0].immediateDisable = True
    # dict_data['carEvents'].carEvents[1].name = 'buttonCancel'
    # dict_data['carEvents'].carEvents[1].userDisable = True

    # # carParams
    # dict_data['carParams'] = messaging.new_message('carParams')
    # dict_data['carParams'].carParams.carName = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['carParams'].carParams.safetyModel = 'chrysler'
    # dict_data['carParams'].carParams.safetyModelPassive = 'silent'
    # dict_data['carParams'].carParams.steerMaxBP = [33.21, 7.65, 67.43]
    # dict_data['carParams'].carParams.longitudinalTuning.kpV = [43.2, 56.2, 564.7]
    # dict_data['carParams'].carParams.lateralParams.torqueBP = [3, 5]
    #
    # # dict_data['carParams'].carParams.lateralTuning.pid = car.CarParams.LateralPIDTuning.new_message()
    # # dict_data['carParams'].carParams.lateralTuning.pid.kiV = [565.2, 565.3]
    # # dict_data['carParams'].carParams.lateralTuning.pid.kpV = [4.2, 66.3]
    #
    # # dict_data['carParams'].carParams.lateralTuning.indi = car.CarParams.LateralINDITuning.new_message()
    # # dict_data['carParams'].carParams.lateralTuning.indi.innerLoopGain = 454.23
    #
    # dict_data['carParams'].carParams.lateralTuning.lqr = car.CarParams.LateralLQRTuning.new_message()
    # dict_data['carParams'].carParams.lateralTuning.lqr.k = [435.1, 5654.1]
    #
    # dict_data['carParams'].carParams.steerControlType = 'angle'
    # dict_data['carParams'].carParams.carVin = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['carParams'].carParams.transmissionType = 'automatic'
    # dict_data['carParams'].carParams.carFw = [car.CarParams.CarFw.new_message(), car.CarParams.CarFw.new_message()]
    # dict_data['carParams'].carParams.carFw[0].ecu = 'srs'
    # dict_data['carParams'].carParams.carFw[0].fwVersion = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['carParams'].carParams.carFw[1].ecu = 'fwdCamera'
    # dict_data['carParams'].carParams.carFw[1].fwVersion = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['carParams'].carParams.fingerprintSource = 'fixed'
    # dict_data['carParams'].carParams.networkLocation = 'fwdCamera'

    # # frontFrame
    # dict_data['frontFrame'] = messaging.new_message('frontFrame')
    # dict_data['frontFrame'].frontFrame.image = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
    # dict_data['frontFrame'].frontFrame.focusVal = [1, 2, 3, -4, -5]
    # dict_data['frontFrame'].frontFrame.focusConf = [0, 128, 255]
    # dict_data['frontFrame'].frontFrame.sharpnessScore = [0, 65535, 500]
    # dict_data['frontFrame'].frontFrame.frameType = 'neo'
    # dict_data['frontFrame'].frontFrame.transform = [-31.56, 12321.954]
    # dict_data['frontFrame'].frontFrame.androidCaptureResult.colorCorrectionTransform = [4, -7, 9]
    # dict_data['frontFrame'].frontFrame.androidCaptureResult.colorCorrectionGains = [4.0, 79.0, -53.4]

    # # dMonitoringState
    # dict_data['dMonitoringState'] = messaging.new_message('dMonitoringState')
    # dict_data['dMonitoringState'].dMonitoringState.events = [car.CarEvent.new_message(), car.CarEvent.new_message()]
    # dict_data['dMonitoringState'].dMonitoringState.events[0].name = 'brakeUnavailable'
    # dict_data['dMonitoringState'].dMonitoringState.events[0].immediateDisable = True
    # dict_data['dMonitoringState'].dMonitoringState.events[1].name = 'buttonCancel'
    # dict_data['dMonitoringState'].dMonitoringState.events[1].userDisable = True
    # dict_data['dMonitoringState'].dMonitoringState.isPreview = True

    # liveLocationKalman
    dict_data['liveLocationKalman'] = messaging.new_message('liveLocationKalman')
    dict_data['liveLocationKalman'].liveLocationKalman.positionECEF.value = [4.0, 79.0, -53.4]
    dict_data['liveLocationKalman'].liveLocationKalman.positionECEF.std = [544.0, 759.450, -6513.4]
    dict_data['liveLocationKalman'].liveLocationKalman.positionGeodetic.value = [4.0, 79.0, -53.4]
    dict_data['liveLocationKalman'].liveLocationKalman.positionGeodetic.std = [544.0, 759.450, -6513.4]
    dict_data['liveLocationKalman'].liveLocationKalman.calibratedOrientationECEF.value = [4.0, 79.0, -53.4]
    dict_data['liveLocationKalman'].liveLocationKalman.calibratedOrientationECEF.std = [544.0, 759.450, -6513.4]
    dict_data['liveLocationKalman'].liveLocationKalman.orientationNEDCalibrated.value = [4.0, 79.0, -53.4]
    dict_data['liveLocationKalman'].liveLocationKalman.orientationNEDCalibrated.std = [544.0, 759.450, -6513.4]
    dict_data['liveLocationKalman'].liveLocationKalman.accelerationCalibrated.value = [4.0, 79.0, -53.4]
    dict_data['liveLocationKalman'].liveLocationKalman.accelerationCalibrated.std = [544.0, 759.450, -6513.4]
    dict_data['liveLocationKalman'].liveLocationKalman.status = 'uncalibrated'
    dict_data['liveLocationKalman'].liveLocationKalman.gpsOK = False

    pubmaster = messaging.PubMaster(dict_data.keys())
    submaster = messaging.SubMaster(dict_data.keys())

    mconv = MessageConverter(list(dict_data.keys()))

    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        for k, d in dict_data.items():
            pubmaster.send(k, d)

        submaster.update()

        mconv.publish(submaster)

        rate.sleep()

    print('Test finished')


def debug():
    import time
    import os
    os.environ["ZMQ"] = "1"

    submaster = messaging.SubMaster(['thermal', 'sensorEvents'], addr="192.168.0.189")

    while True:
        submaster.update()

        for k, v in submaster.updated.items():
            print('{} : {}'.format(k, v))

            if v:
                d = submaster.data[k]
                print(d.to_dict())

            time.sleep(0.5)

    print('Debug finished')


def run():
    rospy.init_node('openpilot')

    freq = rospy.get_param('~freq', 100)
    ip = rospy.get_param('~ip', '192.168.0.189')

    rospy.loginfo("Frequency : {}".format(freq))
    rospy.loginfo("Connect to {}".format(ip))

    list_topic = []
    for k, v in MessageConverter.dict_type.items():
        if '_element' in k:
            continue
        list_topic.append(k)

    submaster = messaging.SubMaster(list_topic, addr=ip)

    mconv = MessageConverter(list_topic)

    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        submaster.update()

        mconv.publish(submaster)

        rate.sleep()

    print('Test finished')





if __name__ == '__main__':
    # test()
    # debug()
    run()
