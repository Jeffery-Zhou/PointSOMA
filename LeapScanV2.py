# -*- coding: UTF-8 -*- 
################################################################################
# Leap Motion to scan workpiece with interaction cube.
# This code is to get the starting position for the scanning camera
# Created by Jeffery on 2020年06月08日20:52:35
################################################################################

import sys
sys.path.append("./lib")

import logging
logging.basicConfig(level = logging.INFO,format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# import for LeapMotion
import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture, Vector

# import for UR3
import urx
import math
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R

# create thread to execute the two UR3 simualtaniously
import threading
from threading import Thread
import time
import copy

from gtts import gTTS 
import os 
# import time
# import copy


# data collection / paper
def addPose(dct, key_a, key_b, val):
    '''
    to call a dict like: dict[a][b] = [....]
    @param dict:
    @param key_a: first key
    @param key_b: second key
    @param val: the element
    ''' 
    if key_a in dct:
        dct[key_a].update({key_b: val})
    else:
        dct.update({key_a:{key_b: val}})

class TTS:
    language = 'en'
    
    @classmethod
    def toSpeech(cls, text, cmd):
        t = gTTS(text = text, lang = cls.language, slow= False)
        t.save("%s.mp3" % cmd)

    @classmethod
    def outSpeech(cls, cmd):
        os.system("mpg321 ./mp/%s.mp3" % cmd)

class printer:   
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @classmethod
    def okb(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.OKBLUE + t + ' ' + text + cls.ENDC)

    @classmethod
    def okg(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.OKGREEN + t + ' ' + text + cls.ENDC)

    @classmethod
    def warn(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.WARNING + t + ' ' + text + cls.ENDC)

    @classmethod
    def header(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.HEADER + t + ' ' + text + cls.ENDC)        

    @classmethod
    def fail(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.FAIL + t + ' ' + text + cls.ENDC)  

    @classmethod
    def bold(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.BOLD + t + ' ' + text + cls.ENDC)          

    @classmethod
    def underline(cls, text):
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(cls.UNDERLINE + t + ' ' + text + cls.ENDC)            

class UR3:
    Welding_UR3_IP = "192.168.0.2"
    grid_xy = [2, 3] 

    @classmethod
    def connectWeldingUR(cls):
        rob = urx.Robot(cls.Welding_UR3_IP)
        rob.set_tcp((0,0,0,0,0,0))
        rob.set_payload(1.5, (0,0,0))
        time.sleep(0.2)
        printer.header("==================== Welding Robot Connected. ====================")
        TTS.outSpeech("robConnect")
        intial_joints = [-1.4590542952166956, -1.5175774733172815, -1.0832245985614222, -2.1050151030169886, 1.5401545763015747, 0.10233666747808456] 
        rob.movej(intial_joints, acc= 0.2, vel=0.1)
        printer.header("==================== Left UR3 finished initial setting. ====================")

        # get the 
        grid_length = 7.0/100
        grid_rows = 4
        grid_columns = 6
        pose_grids = {}
        try:
            bgn_pose = rob.getl()
            # bgn_pose = [-0.06770, -0.41835, 0.6339]
            # t = rob.get_pose() # transformation from Base to TCP
            print("Weld UR: the current pose is : %s"  % bgn_pose)
            for i in range(grid_rows+1):
                for j in range(grid_columns+1):
                    temp_pose = copy.deepcopy(bgn_pose)
                    temp_pose[0] = temp_pose[0] + (j-3)*grid_length # x
                    temp_pose[1] = temp_pose[1] + (i-2)*grid_length # y
                    addPose(pose_grids, i, j, temp_pose)
                    # print("Weld UR: the temp pose is : (%s, %s) %s"  % (str(i), str(j), temp_pose))
            printer.okg("Weld UR: ************** Scanning Surface Generate Successfully! ***************")
            TTS.outSpeech("scanSurf")
            time.sleep(0.0001)
            TTS.outSpeech("LMready")

        finally:
            # rob.close()
            print("The UR3 robot is disconnected!")
            # return pose_grids        
        return (rob,pose_grids)

    @classmethod 
    def moveWeldPose(cls, rob, pos_displace, grid, angle = 0.0):
        x_threshold = 15
        y_threshold = 15
        try:
            l = 0.02
            v = 0.1    # velocity
            a = 0.2     # acceleration
            pose = rob.getl()
            t = rob.get_pose() # transformation from Base to TCP
            # print("Right UR: the current pose is : %s"  % pose)
            flag = 0

            printer.warn(" *-* Current grid [%s,%s]" % (str(cls.grid_xy[0]), str(cls.grid_xy[1])))
  
            y_delta = 0 if abs(pos_displace[0]) <= x_threshold else int(pos_displace[0]/abs(pos_displace[0]))
            x_delta = 0 if abs(pos_displace[1]) <= y_threshold else -int(pos_displace[1]/abs(pos_displace[1]))
            x_pred = cls.grid_xy[0] + x_delta
            y_pred = cls.grid_xy[1] + y_delta
            if x_pred >= 0  and x_pred <=4 and y_pred >= 0 and y_pred <=6:
                if x_delta <> 0 or y_delta <> 0:
                    cls.grid_xy[0] = x_pred
                    cls.grid_xy[1] = y_pred
                    flag = 1
                else:
                    # TTS.outSpeech("notMov")
                    printer.warn("~~~~~~~~~ Hand Gesture Ignored! ~~~~~~~~~")
            else:
                TTS.outSpeech("reachBnd")
                printer.warn("~~~~~~~~~ REACH BUNDARY! ~~~~~~~~~") 
                    
            print("Right UR: the target pose is %s with a displacement %s" % (pose, pos_displace))
            if flag:
                printer.warn(" *-* Moving grid [%s,%s]" % (str(cls.grid_xy[0]), str(cls.grid_xy[1])))
                # TTS.outSpeech("movGrid")
                rob.movel(grid[cls.grid_xy[0]][cls.grid_xy[1]], vel=v, acc=a)
                time.sleep(0.001)
                # print("Right UR: the current pose is : %s"  % rob.getl())
                printer.okg("Right UR: ************** Right UR Moved Successfully! ***************")
            else:
                # printer.warn("") 
                pass           

        finally:
            pass
            # rob.close()
            # print("The UR3 robot is disconnected")
    
    @classmethod 
    def moveCameraPose(cls, rob, direction, pitch, yaw, roll):
        '''
        @param robot object
        @param direction
        @param rotation

        '''

        try:
            # Best step params
            # l = 0.1
            # v = 0.25    # velocity
            # a = 0.1     # acceleration

            l = 0.05
            v = 0.1    # velocity
            a = 0.08     # acceleration


            # l = 0.1
            # v = 0.2    # velocity
            # a = 0.1     # acceleration


            r = 0.005
            pose = rob.getl()
            x = pose[0]
            y = pose[1]
            z = pose[2]
            # t = rob.get_pose() # transformation from Base to TCP
            # print("Right UR: the current pose is : %s"  % pose)
            rx = pose[3]
            ry = pose[4]
            rz = pose[5]

            pitch = int(pitch*0.6) 
            yaw = int(yaw*0.6)
            roll = int(roll*0.6)

            if direction == "home":
                # Handle rotation
                # pitch
                dgr_lst = [yaw, -pitch, (roll/abs(roll))*180-roll]
                # dgr_lst = [yaw, 1.57620936, -179.93848198]
                # dgr_lst = [0.36914324, -pitch,   -178.33805356]
                # dgr_lst = [-0.29658773, 0.35630111, (roll/abs(roll))*180-roll]
                r = R.from_euler('zxy', dgr_lst, degrees=True)
                rxyz = r.as_rotvec()
                rx = rxyz[0]
                ry = rxyz[1]
                rz = rxyz[2]
                flag = 1
                             
            elif direction == "forward":
                y += l
                flag = 1
            elif direction == "backward":
                y -= l
                flag = 1
            elif direction == "left":
                x -= l
                flag = 1
            elif direction == "right":
                x += l
                flag = 1
            elif direction == "up":
                z += l
                flag = 1
            elif direction == "down":
                z -= l
                flag = 1
            else:
                flag = 0

            print("Welding UR: the target pose [%s, %s, %s, %s, %s, %s]" % (x, y, z, rx, ry, rz))
            if flag and rob.is_program_running():
                # rob.stopj(a=0.5)
                # rob.stopj(acc=1.5)
                # acc=1.5
                rob.movel((x, y, z, rx, ry, rz), vel=v, acc=a)
                # time.sleep(0.001)
                printer.okg("Right UR: ************** Right UR Moved Successfully! ***************")
            elif flag:
                rob.movel((x, y, z, rx, ry, rz), vel=v, acc=a)
                # printer.warn("") 
                pass           

        finally:
            # rob.stopj()
            pass
            # rob.close()
            # print("The UR3 robot is disconnected")


class LMController:
    """
    Leap motion interaction control box
    """
    # center_dist = # the distance between the hand palm position to the original point of LM
    # grid_length

    pass

class BoxRegion:

    def __init__(self, bwx, bhy, bdz, dbb):
        self.bwx = bwx
        self.bhy = bhy
        self.bdz = bdz
        self.dbb = dbb

    def getDirction(self, x, y, z):
        if (-self.bwx/2 <= x and x <= self.bwx/2) and (self.dbb <= y and y <= self.dbb + self.bhy) and (-self.bdz/2 <= z and z <= self.bdz/2):
            return "home"

        elif (-self.bwx/2 <= x and x <= self.bwx/2) and (self.dbb <= y and y <= self.dbb + self.bhy) and (z >= self.bdz/2):
            return "backward"

        elif (-self.bwx/2 <= x and x <= self.bwx/2) and (self.dbb <= y and y <= self.dbb + self.bhy) and (-self.bdz/2 >= z):
            return "forward"

        elif (-self.bwx/2 >= x) and (self.dbb <= y and y <= self.dbb + self.bhy) and (-self.bdz/2 <= z and z <= self.bdz/2):
            return "left"

        elif (x >= self.bwx/2) and (self.dbb <= y and y <= self.dbb + self.bhy) and (-self.bdz/2 <= z and z <= self.bdz/2):
            return "right"

        elif (-self.bwx/2 <= x and x <= self.bwx/2) and (y >= self.dbb + self.bhy) and (-self.bdz/2 <= z and z <= self.bdz/2):
            return "up"

        elif (-self.bwx/2 <= x and x <= self.bwx/2) and (self.dbb >= y) and (-self.bdz/2 <= z and z <= self.bdz/2):
            return "down"
        else:
            return "others"

    def toRadians(self, dgr):
        return math.radians(dgr / math.pi)

    def toDegree(self, rad):
        return math.degrees(rad)






class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    logger = logging.getLogger(__name__)

    returns = UR3.connectWeldingUR() # set debug without UR robot connection
    weldrob = returns[0]
    # pose_grids = returns[1]
    # print("get returns")




    def on_init(self, controller):
        printer.header("======================== LeapMotion Initialized ========================")
        # Weldrob = UR3.connectWeldingUR()
        box_wideth_x = 100 
        box_height_y = 100 
        box_depth_z = 100 
        dist_box_bottom = 200
        global box
        box = BoxRegion(box_wideth_x, box_height_y, box_depth_z, dist_box_bottom)

    def on_connect(self, controller):
        printer.okg("======================== Connected ========================")
        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        printer.warn("======================== Leap Motion Disconnected ========================")
        self.weldrob.close()
        printer.header("The UR3 left robot is disconnected")
        # self.rightrob.close()
        # printer.header("The UR3 right robot is disconnected")

    def on_exit(self, controller):
        printer.warn("========================  Leap Motion Listerner Exited ========================")
        self.weldrob.close()
        printer.header("The UR3 left robot is disconnected. ")
        # self.rightrob.close()
        # printer.header("The UR3 right robot is disconnected. ")


    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        cur_frame = controller.frame() # the current frame
        pre_frame = controller.frame(29) # the previous frame 1~59

        """
        # A unique ID assigned to this Hand object, 
        # whose value remains the same across consecutive frames while the tracked hand remains visible. 
        # If tracking is lost (for example, when a hand is occluded by another hand or 
        # when it is withdrawn from or reaches the edge of the Leap Motion Controller field of view), 
        # the Leap Motion software may assign a new ID when it detects the hand in a future frame.
        """


        if cur_frame.id % 60 == 0:
            fps = cur_frame.current_frames_per_second
            # print("Current FPS: %d " % int(fps))
            # print("Current Frame  id: %d, hands: %d, gestures: %d" % (cur_frame.id, len(cur_frame.hands), len(cur_frame.gestures())))
            # print("Previous Frame id: %d, hands: %d, gestures: %d" % (pre_frame.id, len(pre_frame.hands), len(pre_frame.gestures())))

            # get current and previous hands
            if len(cur_frame.hands) == 1 and len(pre_frame.hands) == 1:
                if cur_frame.hands[0].is_left:
                    left_hand_c = cur_frame.hands[0]
                else:
                    pass

                if pre_frame.hands[0].is_left:
                    left_hand_p = pre_frame.hands[0]
                else:
                    pass

                print('') # separate the log
                print("Current Frame  id: %d, hands: %d, gestures: %d" % (cur_frame.id, len(cur_frame.hands), len(cur_frame.gestures())))
                printer.header(" ========================= Current Frame  id: %d, hands: %d ========================= " % (cur_frame.id, len(cur_frame.hands)))
                # print("Current   Left Hand %s, %s, %s" % (left_hand_c.palm_position, left_hand_c.direction, left_hand_c.palm_normal))
                # print("Previous  Left Hand %s, %s, %s" % (left_hand_p.palm_position, left_hand_p.direction, left_hand_p.palm_normal))
                left_c_norm = left_hand_c.palm_normal
                left_p_norm = left_hand_p.palm_normal
                # lh_displace = left_hand_c.palm_position - left_hand_p.palm_position
                cur_palm_postion = left_hand_c.palm_position
                drct = box.getDirction(cur_palm_postion[0], cur_palm_postion[1], cur_palm_postion[2])
                # left_pos_displace = [lh_displace[0], lh_displace[2], lh_displace[1]]

                pitch = left_hand_c.direction.pitch  # z
                yaw = left_hand_c.direction.yaw # x
                roll = left_hand_c.palm_normal.roll # y 

                # printer.warn("LeapMotion: palm normal")
                # print(left_c_norm)
                roll_degree =  box.toDegree(roll)
                pitch_degree = box.toDegree(pitch)
                yaw_degree = box.toDegree(yaw)
                printer.warn("LeapMotion: pitch: %s, yaw: %s, roll: %s" % (roll_degree, pitch_degree, yaw_degree))
                printer.warn("LeapMotion: the left hand moving direction is %s" % drct)

                # %s" % left_c_norm

                if left_c_norm[1] > 0:
                    printer.fail("Leap Motion: Please put your hand palm toward ground and hide UR3 job !")
                else:
                    UR3.moveCameraPose(self.weldrob, drct, pitch_degree, yaw_degree, roll_degree)
                    # printer.okg("Left UR starts to run!")
                    # if self.weldrob.is_program_running():
                    #     # self.weldrob.stop()
                    #     pass
                    # else:
                    #     UR3.moveCameraPose(self.weldrob, drct, pitch, yaw, roll)
                    #     printer.okg("Left UR starts to run!")
                

                # ============ start of UR3 executor ================
                """ 
                # for real time performance
                threads = []
                threads.append(threading.Thread(target=UR3.moveLeftPose(self.leftrob, left_pos_displace)))
                threads.append(threading.Thread(target=UR3.moveRightPose(self.rightrob, right_pos_displace)))

                # Check the robot is runing
                if self.leftrob.is_program_running():
                    pass
                else:
                    threads[0].start()
                    printer.okb("Left UR thread starts to run!")

                if self.rightrob.is_program_running():
                    pass
                else:
                    threads[1].start()
                    printer.okb("Right UR thread starts to run!")
                
                """  
                # =========== end of UR3 excutor ================
            else:
                pass
            # two hands end
                

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"
 
        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"
# End of SampleListener for LeapMotion

def runLeapMotion():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()
    # Have the sample listener receive events from the controller
    controller.add_listener(listener)
    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

def addPose(dct, key_a, key_b, val):
    '''
    to call a dict like: dict[a][b] = [....]
    @param dict:
    @param key_a: first key
    @param key_b: second key
    @param val: the element
    ''' 
    if key_a in dct:
        dct[key_a].update({key_b: val})
    else:
        dct.update({key_a:{key_b: val}})

def buildScanSurface():
    """
    @param 
    """
    # rob = UR3.connectWeldingUR()
    grid_length = 10.0/100
    grid_rows = 4
    grid_columns = 6
    pose_grids = {}
    try:
        # bgn_pose = rob.getl()
        bgn_pose = [-0.06770, -0.41835, 0.6339]
        # t = rob.get_pose() # transformation from Base to TCP
        print("Weld UR: the current pose is : %s"  % bgn_pose)
        for i in range(grid_rows):
            for j in range(grid_columns):
                temp_pose = copy.deepcopy(bgn_pose)
                temp_pose[0] = temp_pose[0] + (j-3)*grid_length # x
                temp_pose[1] = temp_pose[1] + (i-2)*grid_length # y
                addPose(pose_grids, i, j, temp_pose)
                print("Weld UR: the temp pose is : %s"  % temp_pose)
        printer.okg("Weld UR: ************** Scanning Surface Generate Successfully! ***************")
        time.sleep(0.0001)

    finally:
        # rob.close()
        print("The UR3 robot is disconnected!")
        return pose_grids

if __name__ == "__main__":
    # TTS.outSpeech("welcome")
    runLeapMotion()
    

