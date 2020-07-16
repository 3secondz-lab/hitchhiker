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


class Event:
    def __init__(self):
        self.logMonoTime = 0
        self.valid = False

        # Data type should be one of the types in the dict_data (or a list of data)
        self.data = None

        self.dict_data = {
            'initData':                 om.InitData,
            'frame':                    om.FrameData,
            'gpsNMEA':                  om.GPSNMEAData,
            'can':                      om.CanData,
            'thermal':                  om.ThermalData,
            'controlsState':            om.ControlsState,
            'model':                    om.ModelData,
            'features':                 om.CalibrationFeatures,
            'sensorEvents':             om.SensorEventData,
            'health':                   om.HealthData,
            'radarState':               om.RadarState,
            'encodeIdx':                om.EncodeIndex,
            'liveTracks':               om.LiveTracks,
            'sendcan':                  om.CanData,
            'logMessage':               om.Text,
            'liveCalibration':          om.LiveCalibrationData,
            'androidLog':               om.AndroidLogEntry,
            'gpsLocation':              om.GpsLocationData,
            'carState':                 om.CarState,
            'carControl':               om.CarControl,
            'plan':                     om.Plan,
            'liveLocation':             om.LiveLocationData,
            'ethernetData':             om.EthernetPacket,
            'navUpdate':                om.NavUpdate,
            'cellInfo':                 om.CellInfo,
            'wifiScan':                 om.WifiScan,
            'androidGnss':              om.AndroidGnss,
            'qcomGnss':                 om.QcomGnss,
            'lidarPts':                 om.LidarPts,
            'procLog':                  om.ProcLog,
            'ubloxGnss':                om.UbloxGnss,
            'clocks':                   om.Clocks,
            'liveMpc':                  om.LiveMpcData,
            'liveLongitudinalMpc':      om.LiveLongitudinalMpcData,
            'navStatus':                om.NavStatus,
            'ubloxRaw':                 sm.Byte,
            'gpsPlannerPoints':         om.GPSPlannerPoints,
            'gpsPlannerPlan':           om.GpsPlannerPlan,
            'applanixRaw':              sm.Byte,
            'trafficEvents':            om.TrafficEvent,
            'liveLocationTiming':       om.LiveLocationData,
            'liveLocationCorrected':    om.LiveLocationData,
            'orbObservation':           om.OrbObservation,
            'gpsLocationExternal':      om.GpsLocationData,
            'location':                 om.LiveLocationData,
            'uiNavigationEvent':        om.UiNavigationEvent,
            'testJoystick':             om.Joystick,
            'orbOdometry':              om.OrbOdometry,
            'orbFeatures':              om.OrbFeatures,
            'applanixLocation':         om.LiveLocationData,
            'orbKeyFrame':              om.OrbKeyFrame,
            'uiLayoutState':            om.UiLayoutState,
            'orbFeaturesSummary':       om.OrbFeaturesSummary,
            'driverState':              om.DriverState,
            'boot':                     om.Boot,
            'liveParameters':           om.LiveParametersData,
            'liveMapData':              om.LiveMapData,
            'cameraOdometry':           om.CameraOdometry,
            'pathPlan':                 om.PathPlan,
            'kalmanOdometry':           om.KalmanOdometry,
            'thumbnail':                om.Thumbnail,
            'carEvents':                om.CarEvent,
            'carParams':                om.CarParams,
            'frontFrame':               om.FrameData,
            'dMonitoringState':         om.DMonitoringState,
            'liveLocationKalman':       om.LiveLocationKalman,
            'sentinel':                 om.Sentinel
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


class MessageConverter:
    def __init__(self, list_service=None):
        '''
        Convert Openpilot services to ROS messages

        :param list_service: List of Openpilot services to subscribe
        '''

        pm = messaging.PubMaster(['sensorEvents'])
        dat1 = messaging.new_message('sensorEvents', size=1)


        d1 = dat1.to_bytes()
        d2 = log.Event.from_bytes(d1)


        sm = messaging.SubMaster(['sensorEvents', 'frame'])

        d1 = sm.data.to_bytes()




        print(service_list)











if __name__ == '__main__':
    # For test
    rospy.init_node('msg_conv_test')

    dict_data = {
        'can': None,
        'thermal': None,
        'sensorEvents': None
    }

    pub1 = rospy.Publisher('~can', om.CanDataList, queue_size=1)
    pub2 = rospy.Publisher('~thermal', om.ThermalData, queue_size=1)
    pub3 = rospy.Publisher('~sensorEvents', om.SensorEventDataList, queue_size=1)

    pubmaster = messaging.PubMaster(dict_data.keys())
    submaster = messaging.SubMaster(dict_data.keys())

    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        d1 = messaging.new_message('can', size=5)
        for i in range(0, len(d1.can)):
            # address : UInt32
            # busTime : UInt16
            # dat : Data
            # src : UInt8
            d1.can[i].address = random.randint(np.iinfo('uint32').min, np.iinfo('uint32').max)
            d1.can[i].busTime = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
            d1.can[i].dat = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
            d1.can[i].src = random.randint(np.iinfo('uint8').min, np.iinfo('uint8').max)

        d2 = messaging.new_message('thermal')
        d2.thermal.cpu0 = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.cpu1 = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.cpu2 = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.cpu3 = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.mem = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.gpu = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.bat = random.randint(np.iinfo('uint32').min, np.iinfo('uint32').max)
        d2.thermal.pa0 = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.freeSpace = random.random()*1e5
        d2.thermal.batteryPercent = random.randint(np.iinfo('int16').min, np.iinfo('int16').max)
        d2.thermal.batteryStatus = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(10))
        d2.thermal.batteryCurrent = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d2.thermal.batteryVoltage = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d2.thermal.usbOnline = bool(random.randint(0, 1))
        d2.thermal.networkType = random.randint(0, 5)
        d2.thermal.offroadPowerUsage = random.randint(np.iinfo('uint32').min, np.iinfo('uint32').max)
        d2.thermal.networkStrength = random.randint(0, 4)
        d2.thermal.fanSpeed = random.randint(np.iinfo('uint16').min, np.iinfo('uint16').max)
        d2.thermal.started = bool(random.randint(0, 1))
        d2.thermal.startedTs = random.randint(np.iinfo('uint64').min, np.iinfo('uint64').max)
        d2.thermal.thermalStatus = random.randint(0, 3)
        d2.thermal.chargingError = bool(random.randint(0, 1))
        d2.thermal.chargingDisabled = bool(random.randint(0, 1))
        d2.thermal.memUsedPercent = random.randint(np.iinfo('int8').min, np.iinfo('int8').max)
        d2.thermal.cpuPerc = random.randint(np.iinfo('int8').min, np.iinfo('int8').max)

        d3 = messaging.new_message('sensorEvents', size=2)

        d3.sensorEvents[0].version = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[0].sensor = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[0].type = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[0].timestamp = random.randint(np.iinfo('int64').min, np.iinfo('int64').max)
        d3.sensorEvents[0].source = random.randint(0, 7)
        d3.sensorEvents[0].acceleration.status = random.randint(np.iinfo('int8').min, np.iinfo('int8').max)
        d3.sensorEvents[0].acceleration.v = [random.random() * 100 for _ in range(10)]

        d3.sensorEvents[1].version = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[1].sensor = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[1].type = random.randint(np.iinfo('int32').min, np.iinfo('int32').max)
        d3.sensorEvents[1].timestamp = random.randint(np.iinfo('int64').min, np.iinfo('int64').max)
        d3.sensorEvents[1].source = random.randint(0, 7)
        d3.sensorEvents[1].light = random.random() * 100

        dict_data = {
            'can': d1,
            'thermal': d2,
            'sensorEvents': d3
        }

        for k, v in dict_data.items():
            if bool(random.randint(0, 1)):
                pubmaster.send(k, v)
            else:
                rospy.loginfo('{} : Not published'.format(k))

        submaster.update()

        if submaster.updated['can'] and submaster.valid['can']:
            m1 = om.CanDataList()

            m1.header.seq = submaster.frame
            m1.header.stamp = rospy.Time.from_sec(submaster.logMonoTime['can'] / 1e9)

            for d in submaster.data['can']:
                can_data = om.CanData()

                can_data.header = m1.header
                can_data.address.data = d.address
                can_data.busTime.data = d.busTime
                can_data.dat.data = d.dat.decode()
                can_data.src.data = d.src

                m1.can.append(can_data)

            pub1.publish(m1)
        else:
            rospy.loginfo('can : not updated')

        if submaster.updated['thermal'] and submaster.valid['thermal']:
            m2 = om.ThermalData()

            m2.header.seq = submaster.frame
            m2.header.stamp = rospy.Time.from_sec(submaster.logMonoTime['thermal'] / 1e9)

            m2.cpu0.data = submaster.data['thermal'].cpu0
            m2.cpu1.data = submaster.data['thermal'].cpu1
            m2.cpu2.data = submaster.data['thermal'].cpu2
            m2.cpu3.data = submaster.data['thermal'].cpu3
            m2.mem.data = submaster.data['thermal'].mem
            m2.gpu.data = submaster.data['thermal'].gpu
            m2.bat.data = submaster.data['thermal'].bat
            m2.pa0.data = submaster.data['thermal'].pa0
            m2.freeSpace.data = submaster.data['thermal'].freeSpace
            m2.batteryPercent.data = submaster.data['thermal'].batteryPercent
            m2.batteryStatus.data = submaster.data['thermal'].batteryStatus
            m2.batteryCurrent.data = submaster.data['thermal'].batteryCurrent
            m2.batteryVoltage.data = submaster.data['thermal'].batteryVoltage
            m2.usbOnline.data = submaster.data['thermal'].usbOnline
            m2.networkType.data = submaster.data['thermal'].networkType.raw
            m2.offroadPowerUsage.data = submaster.data['thermal'].offroadPowerUsage
            m2.networkStrength.data = submaster.data['thermal'].networkStrength.raw
            m2.fanSpeed.data = submaster.data['thermal'].fanSpeed
            m2.started.data = submaster.data['thermal'].started
            m2.startedTs.data = submaster.data['thermal'].startedTs
            m2.thermalStatus.data = submaster.data['thermal'].thermalStatus.raw
            m2.chargingError.data = submaster.data['thermal'].chargingError
            m2.chargingDisabled.data = submaster.data['thermal'].chargingDisabled
            m2.memUsedPercent.data = submaster.data['thermal'].memUsedPercent
            m2.cpuPerc.data = submaster.data['thermal'].cpuPerc

            pub2.publish(m2)
        else:
            rospy.loginfo('thermal : not updated')

        if submaster.updated['sensorEvents'] and submaster.valid['sensorEvents']:
            m3 = om.SensorEventDataList()

            m3.header.seq = submaster.frame
            m3.header.stamp = rospy.Time.from_sec(submaster.logMonoTime['sensorEvents'] / 1e9)

            s1 = om.SensorEventData()
            s1.header = m3.header
            s1.version.data = submaster.data['sensorEvents'][0].version
            s1.sensor.data = submaster.data['sensorEvents'][0].sensor
            s1.type.data = submaster.data['sensorEvents'][0].type
            s1.timestamp.data = submaster.data['sensorEvents'][0].timestamp
            s1.source.data = submaster.data['sensorEvents'][0].source.raw
            for v in submaster.data['sensorEvents'][0].acceleration.v:
                v_tmp = sm.Float32()
                v_tmp.data = v
                s1.acceleration.v.append(v_tmp)
            s1.acceleration.status.data = submaster.data['sensorEvents'][0].acceleration.status

            s2 = om.SensorEventData()
            s2.header = m3.header
            s2.version.data = submaster.data['sensorEvents'][1].version
            s2.sensor.data = submaster.data['sensorEvents'][1].sensor
            s2.type.data = submaster.data['sensorEvents'][1].type
            s2.timestamp.data = submaster.data['sensorEvents'][1].timestamp
            s2.source.data = submaster.data['sensorEvents'][1].source.raw
            s2.light.data = submaster.data['sensorEvents'][1].light

            m3.sensorEvents = [s1, s2]

            pub3.publish(m3)
        else:
            rospy.loginfo('sensorEvents : not updated')

        rate.sleep()

    print('Test finished')
