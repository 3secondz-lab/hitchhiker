#!/usr/bin/python3
"""
    Openpilot-ROS message converter

    Jinsun Park
    (zzangjinsun@3secondz.com)
"""


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
from cereal import log
from capnp.lib import capnp


class MessageConverter:
    def __init__(self, list_service):
        """
        Convert Openpilot services to ROS messages

        :param list_service: List of Openpilot services to subscribe
        """

        self.dict_type = {
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
            'sendcan': om.CanDataList,
            'sendcan_element': om.CanData,
            'logMessage': sm.String,
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
            'cellInfo': om.CellInfoList,
            'cellInfo_element': om.CellInfo,
            'wifiScan': om.WifiScanList,
            'wifiScan_element': om.WifiScan,
            'androidGnss': om.AndroidGnss,
            'qcomGnss': om.QcomGnss,
            'lidarPts': om.LidarPts,
            'procLog': om.ProcLog,
            'ubloxGnss': om.UbloxGnss,
            'clocks': om.Clocks,
            'liveMpc': om.LiveMpcData,
            'liveLongitudinalMpc': om.LiveLongitudinalMpcData,
            'navStatus': om.NavStatus,
            'ubloxRaw': sm.Byte,
            'gpsPlannerPoints': om.GPSPlannerPoints,
            'gpsPlannerPlan': om.GPSPlannerPlan,
            'applanixRaw': sm.Byte,
            'trafficEvents': om.TrafficEventList,
            'trafficEvents_element': om.TrafficEvent,
            'liveLocationTiming': om.LiveLocationData,
            'liveLocationCorrected': om.LiveLocationData,
            'orbObservation': om.OrbObservationList,
            'orbObservation_element': om.OrbObservation,
            'gpsLocationExternal': om.GpsLocationData,
            'location': om.LiveLocationData,
            'uiNavigationEvent': om.UiNavigationEvent,
            'testJoystick': om.Joystick,
            'orbOdometry': om.OrbOdometry,
            'orbFeatures': om.OrbFeatures,
            'applanixLocation': om.LiveLocationData,
            'orbKeyFrame': om.OrbKeyFrame,
            'uiLayoutState': om.UiLayoutState,
            'orbFeaturesSummary': om.OrbFeaturesSummary,
            'driverState': om.DriverState,
            'boot': om.Boot,
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
            'sentinel': om.Sentinel
        }

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
        else:
            print('Unsupported type : {}'.format(type(d)))

        return m

    def convert_dict(self, msg, dict_data):
        for k, v in dict_data.items():
            # print('{} : {}'.format(k, v))
            if 'deprecated' in k.lower():
                continue

            if type(v) == list:
                setattr(msg, k, v)
            elif type(v) == dict:
                if hasattr(msg, k):
                    att = getattr(msg, k)
                    self.convert_dict(att, v)
                else:
                    for k2, v2 in v.items():
                        att = getattr(msg, k2)
                        self.convert_dict(att, v2)
            else:
                att = getattr(msg, k)
                setattr(att, 'data', v)

        return msg


def test():
    # For test
    rospy.init_node('msg_conv_test')

    dict_data = {
        # 'frame': None,
        # 'gpsNMEA': None,
        # 'can': None,
        # 'thermal': None,
        # 'controlsState': None,
        # 'model': None,
        # 'features': None,
        # 'sensorEvents': None,
        'health': None,
    }

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

    # health
    dict_data['health'] = messaging.new_message('health')
    dict_data['health'].health.voltage = 20000
    dict_data['health'].health.usbPowerMode = 'cdp'
    dict_data['health'].health.hwType = 'greyPanda'
    dict_data['health'].health.faultStatus = 'faultTemp'
    dict_data['health'].health.faults = ['interruptRateGmlan', 'interruptRateUsb']



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


if __name__ == '__main__':
    test()
