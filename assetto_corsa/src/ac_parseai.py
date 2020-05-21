import traceback
import struct
import logging
import json
import numpy as np
from copy import deepcopy
import sys

class ai(object):
    def __init__(self, ac_path, fname_track, logger):

        if sys.platform== 'win32':
            self.fname = ac_path + "\\content\\tracks\\" + fname_track + '\\ai\\fast_lane.ai'
        else:
            self.fname = ac_path + "/content/tracks/" + fname_track + '/ai/fast_lane.ai'

        print(self.fname)
        self.logger = logger
        self.ctl = {}
        self.preview = {'plane':{}, 'cam':{}}
        self.read()
    
    def read(self):
        self.logger.info("Loading path file...")
        self.logger.info(self.fname)
        self.data = open(self.fname,'rb')
        self.header = struct.unpack('IIII',self.data.read(16))
        
        self.logger.info("Parsing file header...")
        self.logger.info(self.header)
        self.length = self.header[1]
        self.line = {}

        self.read_line()
        self.read_bound()
        self.get_euler()
        self.get_bound()
        self.preview = {'plane':{'cte':0, 'psie':0}, 'cam':{} }
    def read_line(self):
        chk = -1
        
        self.line['center_x'] = [0]*self.length
        self.line['center_y'] = [0]*self.length
        self.line['distance'] = [0]*self.length
        
        for i in range(self.length):
            tmp = struct.unpack('ffffI',self.data.read(20))
            self.line['center_x'][i] = tmp[0]
            self.line['center_y'][i] = tmp[2]
            self.line['distance'][i] = tmp[3]
            try:
                if tmp[4]-chk != 1:
                    raise ValueError("corrupted sequence number")
            except ValueError as ve:
                print(ve)
            chk = tmp[4]
        self.totallength = self.line['distance'][-1] + np.sqrt((self.line['center_x'][0]-self.line['center_x'][-1])**2+(self.line['center_y'][0]-self.line['center_y'][-1])**2)
        self.line['distance_normalized'] = np.array(self.line['distance'])/self.totallength

    def read_bound(self):
        self.line['unknown_0'] = [0]*self.length
        self.line['speed'] = [0]*self.length
        self.line['accel'] = [0]*self.length
        self.line['unknown_3'] = [0]*self.length
        self.line['yawrate'] = [0]*self.length
        self.line['radius'] = [0]*self.length
        self.line['off_left']= [0]*self.length
        self.line['off_right'] = [0]*self.length
        self.line['unknown_8'] = [0]*self.length
        self.line['unknown_9'] = [0]*self.length
        self.line['gravity_x'] = [0]*self.length
        self.line['gravity_z'] = [0]*self.length
        self.line['gravity_y'] = [0]*self.length
        self.line['unknown_13'] = [0]*self.length
        self.line['vector_1'] = [0]*self.length
        self.line['vector_2'] = [0]*self.length
        self.line['vector_3'] = [0]*self.length
        self.line['unknown_17'] = [0]*self.length

        for i in range(self.length):
            tmp = struct.unpack('f'*18,self.data.read(72))
            self.line['unknown_0'][i] = tmp[0]
            self.line['speed'][i] = tmp[1]
            self.line['accel'][i] = tmp[2]
            self.line['unknown_3'][i] = tmp[3]
            self.line['yawrate'][i] = tmp[4]
            self.line['radius'][i] = tmp[5]
            self.line['off_left'][i]= tmp[6]
            self.line['off_right'][i] = tmp[7]
            self.line['unknown_8'][i] = tmp[8]
            self.line['unknown_9'][i] = tmp[9]
            self.line['gravity_x'][i] = tmp[10]
            self.line['gravity_z'][i] = tmp[11]
            self.line['gravity_y'][i] = tmp[12]
            self.line['unknown_13'][i] = tmp[13]
            self.line['vector_1'][i] = tmp[14]
            self.line['vector_2'][i] = tmp[15]
            self.line['vector_3'][i] = tmp[16]
            self.line['unknown_17'][i] = tmp[17]

        for key, value in self.line.items():
            self.line[key] = np.array(value)

    def get_euler(self):
        x = self.line['vector_1']
        y = self.line['vector_3']
        z = self.line['vector_2']
        gx = self.line['gravity_x']
        gy = self.line['gravity_y']
        gz = self.line['gravity_z']
        self.line['psi'] = np.arctan2(y,x)
        self.line['theta'] = np.arcsin(gy/gz)
        self.line['phi'] = gx/gz

    def get_bound(self):
        x = self.line['center_x']
        y = self.line['center_y']
        yaw = self.line['psi']
        off_left = self.line['off_left']
        off_right = self.line['off_right']

        bound_lx = x + np.sin(yaw)*off_left
        bound_ly = y - np.cos(yaw)*off_left
        bound_rx = x - np.sin(yaw)*off_right
        bound_ry = y + np.cos(yaw)*off_right

        dist_l = list(np.cumsum(np.sqrt(np.gradient(bound_lx)**2 + np.gradient(bound_ly)**2))).pop()
        dist_r = list(np.cumsum(np.sqrt(np.gradient(bound_rx)**2 + np.gradient(bound_ry)**2))).pop()

        if dist_r > dist_l:
            self.line['inner_x'] = bound_lx
            self.line['inner_y'] = bound_ly
            self.line['outer_x'] = bound_rx
            self.line['outer_y'] = bound_ry
            self.left = 'inner'
            self.right = 'outer'
        else:
            self.line['inner_x'] = bound_rx
            self.line['inner_y'] = bound_ry
            self.line['outer_x'] = bound_lx
            self.line['outer_y'] = bound_ly
            self.left = 'outer'
            self.right = 'inner'

    def recv_telemetry(self, tele):
        if not hasattr(self, 'tele'):
            self.tele = None
        if not hasattr(self, 'prev'):
            self.prev = None
        try:
            self.prev = deepcopy(self.tele)
            self.tele = deepcopy(tele)
        except Exception:
            traceback.print_exc()
        try:
            self.get_heading()
            idx = self.get_position()
            self.get_preview(idx, 50, 0)
        except Exception:
            traceback.print_exc()
            pass

    def get_heading(self):
        dx = self.tele.carCoordinates[0] - self.prev.carCoordinates[0]
        dy = self.tele.carCoordinates[2] - self.prev.carCoordinates[2]
        # print(dx, dy)
        self.ctl['psi'] = np.arctan2(dy,dx)

    def get_position(self):
        idx = np.searchsorted(self.line['distance_normalized'], self.tele.carPositionNormalized, side="left")
        return idx

    def get_preview(self, current_index, preview_length, transform_matrix):
        # window = list(range(current_index, current_index+preview_length))

        sp = current_index
        ep = np.searchsorted(self.line['distance_normalized'], self.tele.carPositionNormalized + preview_length/self.totallength, side="left")

        # print( self.tele.carPositionNormalized,self.ctl['psi'],self.line['distance_normalized'][sp])
        window = list(range(sp, ep))

        center_x = self.line['center_x'][window]
        center_y = self.line['center_y'][window]
        left_x = self.line[self.left+'_x'][window]
        left_y = self.line[self.left+'_y'][window]
        right_x = self.line[self.right+'_x'][window]
        right_y = self.line[self.right+'_y'][window]
        center = [self.tele.carCoordinates[0], self.tele.carCoordinates[2]]
        center_x = center_x - center[0]
        center_y = center_y - center[1]
        left_x = left_x - center[0]
        left_y = left_y - center[1]
        right_x = right_x - center[0]
        right_y = right_y - center[1]
        speed = self.line['speed'][window]

        yaw = self.ctl['psi']

        self.preview['plane']['radius'] = self.line['radius'][window]


        self.preview['plane']['center_x'] = np.cos(yaw)*center_x + np.sin(yaw)*center_y
        self.preview['plane']['center_y'] = -np.sin(yaw)*center_x + np.cos(yaw)*center_y
        self.preview['plane']['left_x'] = np.cos(yaw)*left_x + np.sin(yaw)*left_y
        self.preview['plane']['left_y'] = -np.sin(yaw)*left_x + np.cos(yaw)*left_y
        self.preview['plane']['right_x'] = np.cos(yaw)*right_x + np.sin(yaw)*right_y
        self.preview['plane']['right_y'] = -np.sin(yaw)*right_x + np.cos(yaw)*right_y
        self.preview['plane']['speed'] = self.line['speed'][window]

        M = np.array([[ 1.89999998e-01,  8.99999976e-02,  5.00000000e-01],
       [-2.22044605e-17,  9.99999975e-02, -4.99999987e-01],
       [-1.73749899e-16,  1.79999995e-01,  1.00000000e+00]])
        center = M.dot(np.vstack(([self.preview['plane']['center_y']],[self.preview['plane']['center_x']],[np.ones((center_x.shape))])))
        left = M.dot(np.vstack(([self.preview['plane']['left_y']],[self.preview['plane']['left_x']],[np.ones((center_x.shape))])))
        right = M.dot(np.vstack(([self.preview['plane']['right_y']],[self.preview['plane']['right_x']],[np.ones((center_x.shape))])))
        # print(perc_center)
        self.preview['cam']['center_x'] = np.divide(center[0],center[2])
        self.preview['cam']['center_y'] = np.divide(center[1],center[2])
        self.preview['cam']['left_x'] = np.divide(left[0],left[2])
        self.preview['cam']['left_y'] = np.divide(left[1],left[2])
        self.preview['cam']['right_x'] = np.divide(right[0],right[2])
        self.preview['cam']['right_y'] = np.divide(right[1],right[2])
    
        self.preview['plane']['psie'] = np.arctan2(self.preview['plane']['center_y'][1]-  self.preview['plane']['center_y'][0],
        self.preview['plane']['center_x'][1]-self.preview['plane']['center_x'][0])
        self.preview['plane']['cte'] = self.preview['plane']['center_y'][0]

if __name__ == "__main__":
    debug_logger = logging.getLogger('dev')
    debug_logger.setLevel(logging.INFO)
    debug_handler = logging.StreamHandler()
    debug_handler.setFormatter(logging.Formatter('[%(asctime)s] %(name)s-%(levelname)s: %(message)s'))
    debug_logger.addHandler(debug_handler)
    debug_logger.info("ac_parseai.py")

    aipath=ai('/mnt/d/Steam/steamapps/common/assettocorsa','imola', debug_logger)
    aipath.get_euler()
    aipath.get_bound()    
    aipath.preview = {'plane':{}, 'cam':{}}
    
    import matplotlib.pyplot as plt

    # aipath.get_preview(0,100,1)

    fig, (ax1, ax2, ax3) = plt.subplots(1,3)
    ax1.set(xlim=(-1000,1000),ylim=(-1000,1000))
    ax2.set(xlim=(-50,50),ylim=(0,100))
    ax3.set(xlim=(-2,2), ylim=(-1,3))
    l11,=ax1.plot([],[])
    l12,=ax1.plot([],[])
    l13,=ax1.plot([],[])
    l14,=ax1.plot([],[],'o')
    l21,=ax2.plot([],[])
    l22,=ax2.plot([],[])
    l23,=ax2.plot([],[])
    l31,=ax3.plot([],[])
    l32,=ax3.plot([],[])
    l33,=ax3.plot([],[])
    l11.set_data(aipath.line['center_x'], aipath.line['center_y'])
    l12.set_data(aipath.line['inner_x'], aipath.line['inner_y'])
    l13.set_data(aipath.line['outer_x'], aipath.line['outer_y'])
    for i in range(aipath.length-100):
        aipath.get_preview(i,100,1)
        
        l14.set_data(aipath.line['center_x'][i],aipath.line['center_y'][i])
        l21.set_data(-aipath.preview['plane']['center_y'], aipath.preview['plane']['center_x'])
        l22.set_data(-aipath.preview['plane']['left_y'], aipath.preview['plane']['left_x'])
        l23.set_data(-aipath.preview['plane']['right_y'], aipath.preview['plane']['right_x'])
        l31.set_data(-aipath.preview['cam']['center_x'], aipath.preview['cam']['center_y'])
        l32.set_data(-aipath.preview['cam']['left_x'], aipath.preview['cam']['left_y'])
        l33.set_data(-aipath.preview['cam']['right_x'], aipath.preview['cam']['right_y'])
        # l11,l12,l13, =ax1.plot(aipath.line['center_x'],aipath.line['center_y'],aipath.line['inner_x'],aipath.line['inner_y'],aipath.line['outer_x'],aipath.line['outer_y'], aipath.line['center_x'][i],aipath.line['center_y'][i],'o')
        # l21,l22,l23, =ax2.plot(-aipath.preview['plane']['center_y'], aipath.preview['plane']['center_x'],-aipath.preview['plane']['left_y'], aipath.preview['plane']['left_x'],-aipath.preview['plane']['right_y'], aipath.preview['plane']['right_x'])
        # l31,l32,l33, =ax3.plot(-aipath.preview['cam']['center_x'], aipath.preview['cam']['center_y'],-aipath.preview['cam']['left_x'], aipath.preview['cam']['left_y'],-aipath.preview['cam']['right_x'], aipath.preview['cam']['right_y'])
    # plt.plot(-aipath.preview['plane']['center_y'], aipath.preview['plane']['center_x'],-aipath.preview['plane']['left_y'], aipath.preview['plane']['left_x'],-aipath.preview['plane']['right_y'], aipath.preview['plane']['right_x'])
    # plt.plot(aipath.line['center_x'],aipath.line['center_y'],aipath.line['inner_x'],aipath.line['inner_y'],aipath.line['outer_x'],aipath.line['outer_y'])
    # plt.plot(-aipath.preview['cam']['center_x'], aipath.preview['cam']['center_y'],-aipath.preview['cam']['left_x'], aipath.preview['cam']['left_y'],-aipath.preview['cam']['right_x'], aipath.preview['cam']['right_y'])
        plt.pause(0.001)
        # plt.draw()
    plt.show()
    # with open('./data.json','w') as fp:
    #     json.dump(aipath.line, fp)
    pass
