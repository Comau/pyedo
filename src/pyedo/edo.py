## pyedo
##
## SP
## @file SDK_python.py
## @author  Comau
## @version 2.0
## @date 15.07.2021
## 

import time
import roslibpy
from array import *
from pynput import keyboard
import queue 
import threading 
import json
from json import encoder
from enum import Enum, unique

@unique
class MoveCommand(Enum):
    EXE_JOGMOVE           = 74  # 'J'
    EXE_JOGSTOP           = 83  # 'S'
    EXE_MOVE              = 77  # 'M'
    PAUSE                 = 80  # 'P' Pause movement execution
    RESUME                = 82  # 'R' Resume movement execution
    CANCEL_MOVE           = 67  # 'C' Cancel movement execution and empty robot queue
                                
@unique                         
class MoveType(Enum):           
    JOINT                 = 74  # 'J'
    LINEAR                = 76  # 'L'
    CIRCULAR              = 67  # 'C'
                                
@unique                         
class DataType(Enum):           
    E_MOVE_POINT_JOINT    = 74  # 'J'
    E_MOVE_POINT_POSITION = 80  # 'P'
    E_MOVE_POINT_XTND_POS = 88  # 'X'

@unique
class MaskType(Enum):
    JOINT_MASK6           = 63  # Joint Mask for 6 joints
    JOINT_MASK7           = 127 # Joint Mask for 6 joints + gripper
    JOINT_MASK7_EXT       = 64  # Joint Mask for 7th axis

@unique
class Delay(Enum):
    FLY                   = 255
    ZERO_DELAY            = 0

class edo(object):
    def __init__(self, host = '192.168.12.1', port = 9090, axes = 7): #default connection via wifi, connection via etehernet with '10.42.0.49'
        self.port = port
        self.host = host
        self.home = []
        self.command = {}
        self.commandTemplate = {
                 'move_command': 0,
                 'move_type': 0,
                 'ovr': 100,
                 'delay': 0,
                 'remote_tool': 0,
                 'cartesian_linear_speed': 0.0,
                 'target': {
                     'data_type': 0,
                     'cartesian_data': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'e': 0.0, 'r': 0.0, 'config_flags': ''},
                     'joints_mask': 0,
                     'joints_data': [0]},
                 'via': {
                     'data_type': 0,
                     'cartesian_data': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'e': 0.0, 'r': 0.0, 'config_flags': ''},
                     'joints_mask': 0,
                     'joints_data': [0]},
                 'tool': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'e': 0.0, 'r': 0.0},
                 'frame': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'a': 0.0, 'e': 0.0, 'r': 0.0}}
        # Methods Properties
        self.verbose = False #T = print a message after each command given to e.DO
        # Joints
        self.jointStateValues = {}
        # KeyButton for JOG-Messages
        #self.Mode = 0
        #self.JointArray = array('f',[0,0,0,0,0,0,0])
        #self.JointArray = [0,0,0,0,0,0,0]
        #self.JointJSON = json.dumps(self.JointArray)
        #self.KeyButton = 0
        #self.Mode = MoveType.JOINT.value
        #self.Joint_Selected = 1
        self.stepByStep = False #T = wait the end of current e.DO movement before moving to next command
        #CONNECTION TO e.DO
        client = roslibpy.Ros(self.host, self.port) # Wi-Fi
        client.run()
        if client.is_connected == False:
            print('WebSocket is busy, please disconnect the e.DO App')
        else:
            self.JointInit = roslibpy.Topic(client, '/bridge_init', 'edo_core_msgs/JointInit')
            self.JointReset = roslibpy.Topic(client, '/bridge_jnt_reset', 'edo_core_msgs/JointReset')
            self.JointCalibration = roslibpy.Topic(client, '/bridge_jnt_calib', 'edo_core_msgs/JointCalibration')
            self.JointState = roslibpy.Topic(client, '/usb_jnt_state', 'edo_core_msgs/JointStateArray',throttle_rate=200)
            self.MovementCommand = roslibpy.Topic(client, '/bridge_move', 'edo_core_msgs/MovementCommand')
            self.MovementFeedback = roslibpy.Topic(client, '/machine_movement_ack', 'edo_core_msgs/MovementFeedback',throttle_rate=200)
            self.JogCommand = roslibpy.Topic(client, '/bridge_jog', 'edo_core_msgs/MovementCommand',queue_size = 1)
            self.MachineState = roslibpy.Topic(client, '/machine_state', 'edo_core_msgs/MachineState')
            self.CartesianPosition = roslibpy.Topic(client, '/cartesian_pose', 'edo_core_msgs/CartesianPose',throttle_rate=200)
            print('Connected with e.DO')
    
    # VERBOSITY MODE ON
    def verboseOn(self):
        self.verbose = True
        print('Verbose mode ON')
    
    # VERBOSITY MODE OFF
    def verboseOff(self):
        self.verbose = False
        print('Verbose mode OFF')
    
    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper)
    def init7Axes(self):
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK7.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command)) 
        print('e.DO with Gripper has been initialized')
    
    # INITIALIZATION of the Robot with 6 Joints
    def init6Axes(self):
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK6.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command)) 
        print('e.DO with 6 Joints has been initialized')

        
    # STANDARD DISENGAGE of the Robot    
    def disengageStd(self):
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 2000,
            'disengage_offset': 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.verbose: print('Standard Disengage')
    
    # SINUSOIDAL DISENGAGE of the Robot 
    def disengageSin(self):
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 1,
            'disengage_offset': 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.verbose : print('Sinusoidal Disengage')
    
    # SAFE DISENGAGE of the Robot without the any disengage movement
    def disengageSafe(self):
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 1,
            'disengage_offset': 0.0
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.verbose: print('Safe Disengage')
    
    #CALIBRATION of all the axes the Robot (Be sure that the notches are aligned)
    def calibAxes(self):
        self.command = {**self.commandTemplate}
        self.command = {"joints_mask": MaskType.JOINT_MASK7.value}
        self.JointCalibration.publish(roslibpy.Message(self.command))
        if self.verbose: print('Calibration Axes')
    
    #SET SPEED (Default = 100%, values bigger than 100 not accepted)
    def setSpeed(self, ovr = 100):
        if ovr > 100: 
            print('ATTENTION. Max speed = 100%.')
            ovr = 100
        elif ovr < 0:
            print('ATTENTION. Min speed = 0%.')
            ovr = 0     
        self.commandTemplate['ovr'] = ovr


    #Move e.DO to vertical position (calibration)
    def moveToHome(self):
        self.command = {**self.commandTemplate}    
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0, 0, 0, 0, 0, 0, 0]

        time.sleep(1)
        if self.stepByStep:
            while self.getJoints()['MachineState'].get('current_state') != 2:
                time.sleep(0.5)

        self.MovementCommand.publish(roslibpy.Message(self.command))
        self.gripperClose()
        if self.verbose: print('Move to Home Position')


    #JOINT MOVE
    #def move_joint(self,ovr=30, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
    def moveJoints(self, j1 = 0.0, j2 = 0.0, j3 = 0.0, j4 = 0.0, j5 = 0.0, j6 = 0.0):
        if (j1 < -178 or j1 > 178):
            print('ERROR. J1 value not acceptable.')
            return
        if j2 < -99 or j2 > 99:
            print('ERROR. J2 value not acceptable.')
            return
        if j3 < -99 or j3 > 99:
            print('ERROR. J3 value not acceptable.')
            return
        if j4 < -178 or j4 > 178:
            print('ERROR. J4 value not acceptable.')
            return
        if j5 < -103 or j5 > 103:
            print('ERROR. J5 value not acceptable.')
            return
        if j6 < -178 or j6 > 178:
            print('ERROR. J6 value not acceptable.')
            return
        else:
            self.command = {**self.commandTemplate}    
            self.command['move_command'] = MoveCommand.EXE_MOVE.value
            self.command['move_type'] = MoveType.JOINT.value
            #self.command['ovr'] = ovr
            self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
            self.command['target']['joints_mask'] = MaskType.JOINT_MASK6.value
            self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6]
        
        #if ovr > 100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
            time.sleep(1)
            if self.stepByStep:
                while self.getJoints()['MachineState'].get('current_state') != 2:
                    time.sleep(0.5)

            self.MovementCommand.publish(roslibpy.Message(self.command))
            if self.verbose: print('Move Joints')


    def moveSingleJoint(self, num, value):
        if num not in range(1,7): 
            print('ERROR. Choose a joint number between 1 and 6.')
            return
        if (num == 1 or num == 4 or num == 6) and (value < -178 or value > 178):
            print('ERROR. Joint value not acceptable.')
            return
        if (num == 2 or num == 3) and (value < -99 or value > 99):
            print('ERROR. Joint value not acceptable.')
            return
        if num == 5 and (value < -103 or value > 103):
            print('ERROR. Joint value not acceptable.')
            return
        else:
            self.command = {**self.commandTemplate}
            self.command['move_command'] = MoveCommand.EXE_MOVE.value
            self.command['move_type'] = MoveType.JOINT.value
            self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
            self.command['target']['joints_mask'] = pow(2, num - 1)
            self.command['target']['joints_data'] = [value, value, value, value, value, value]

            time.sleep(1)
            if self.stepByStep:
                while self.getJoints()['MachineState'].get('current_state') != 2:
                    time.sleep(0.5)
        
            self.MovementCommand.publish(roslibpy.Message(self.command))
            if self.verbose: print('Move Single Joint')
   
    #CARTESIAN MOVE  
    #def move_cartesian(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0):
    def moveCartesian(self, x = 370, y = 0, z = 210, a = 0, e = 180, r = 0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {'x': x, 'y': y, 'z': z, 'a': a, 'e': e, 'r': r, 'config_flags': ''}
        
        #if ovr > 100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        time.sleep(1)
        if self.stepByStep:
            while self.getJoints()['MachineState'].get('current_state') != 2:
                time.sleep(0.5)

        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose: print('Move Cartesian')    


    #GRIPPER MOVE
    #def move_gripper(self,ovr=30, j7=0.0):
    def moveGripper(self,j7=0.0):
        self.command = {**self.commandTemplate}
        # Stroke end check
        if j7 < 0: 
            print('ATTENTION. Min gripper width = 0 mm.')
            print('Gripper will close to 0 mm.')
            j7 = 0
        elif j7 > 80:
            print('ATTENTION. Max gripper width = 80 mm.')  
            print('Gripper will open to 80 mm.')
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0, 0, 0, 0, 0, 0, j7]
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Gripper')
        
        
        
    def gripperWidth(self, toInteger = True): # if FALSE = return gripper width in float with 14 decimal values
        self.listenValues()
        time.sleep(1)
        if toInteger:
            return int(self.jointStatePosition[6])
        else:
            return self.jointStatePosition[6]
    


    
    #JOINT JOG MOVE  
    #def jogJoint(self,ovr=100, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
    def jogJoint(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        #self.command['ovr'] = ovr
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6, j7]
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Jog Joint')
    
    #STOP JOG
    def jogStop(self):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGSTOP.value
        self.command['delay'] = Delay.FLY.value
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Jog Stop')
    
    #CARTESIAN JOG MOVE  
    #def jogCartesian(self,ovr=100, x=0, y=0, z=0, a=0, e=0, r=0):
    def jogCartesian(self, x=0, y=0, z=0, a=0, e=0, r=0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        #self.command['ovr'] = ovr
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Jog Cartesian')
     
    #EXTENDED MOVE  
    #def moveCartesianX(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0):
    def moveCartesianX(self, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cartesian Extended')
    
    #CIRCULAR MOVE
    #def moveCircular(self,ovr=30, x1=447, y1=0, z1=180, a1=0, e1=180, r1=0, x2=0, y2=432, z2=15, a2=180, e2=0, r2=180):
    def moveCircular(self, x1=447, y1=0, z1=180, a1=0, e1=180, r1=0, x2=0, y2=432, z2=15, a2=180, e2=0, r2=180):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Circular')
    
    #CIRCULAR MOVE EXTENDED
    #def moveCircularX(self,ovr=30, x1=480, y1=23, z1=640, a1=-12, e1=108, r1=0, j71=0, x2=480, y2=23, z2=640, a2=-12, e2=108, r2=0, j72=0):
    def moveCircularX(self, x1=480, y1=23, z1=640, a1=-12, e1=108, r1=0, j71=0, x2=480, y2=23, z2=640, a2=-12, e2=108, r2=0, j72=0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j71]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j72]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Circular Extended')
    
    #CANCEL MOVE
    def moveCancel(self):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.CANCEL_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cancel')

    #COOPERATIVE MOVE
    #def move_cooperative(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
    def moveCooperative(self,x=480, y=23, z=640, a=-12, e=108, r=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cooperative')
        
    #COOPERATIVE EXTENDED MOVE
    #def moveCooperativeX(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
    def moveCooperativeX(self, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        #self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        #if ovr>100:
        #    self.command['ovr'] = 100
        #    print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cooperative Extended')
        

    #FILL VALUES
    def fillJointStateValues(self, message):
        self.joints = message['joints'];
        self.jointStatePosition = []
        self.jointStateVelocity = []
        self.jointStateCurrent  = []
        for vi_Idx in range(len(self.joints)):
            self.jointStatePosition.append(self.joints[vi_Idx]['position'])
            self.jointStateVelocity.append(self.joints[vi_Idx]['velocity'])
            self.jointStateCurrent.append(self.joints[vi_Idx]['current'])
        
        self.jointStateValues['jointPosition'] = self.jointStatePosition
        self.jointStateValues['velocity']      = self.jointStateVelocity
        self.jointStateValues['currrent']      = self.jointStateCurrent
    
    # Print Joint values [deg] rounded to the 2nd decimal
    def getJoints(self):
        return [round(float(i),2) for i in self.jointStatePosition[0:-1]]
    # Print Cartesian X Y Z values [mm] rounded to the 2th decimal
    def getCartesian(self):
        return [round(float(i),2) for i in self.jointStateValues['cartesianPosition'][0:3]]
    # Print Cartesian values rounded to the 2th decimal
    def getCartesianFull(self):
        return [round(float(i),2) for i in self.jointStateValues['cartesianPosition']]
    # Print gripper opening value in [mm] rounded to the 2nd decimal
    def getGripper(self): #TBD leggere da app_jnt_state -> AppStateArray
        return round(float(self.jointStatePosition[6]),2)
        
    #LISTEN_MOVEMENT_ACK
    def listenMovementAck(self):
        self.MovementFeedback.subscribe(self.fillMovementAck)
        if self.verbose : print('Subscribed to Movement Acknowledge')
    #LISTEN_CARTESIAN_POSE
    def listenCartesianPosition(self):
        self.CartesianPosition.subscribe(self.fillCartesianPosition)
        if self.verbose : print('Subscribed to Cartesian Position')
    #LISTEN_JOINT_STATE
    def listenJointState(self):
        #self.JointState.subscribe(lambda message: print('messaggio = ' + message['data']))
        self.JointState.subscribe(self.fillJointStateValues)
        if self.verbose : print('Subscribed to Joint Position')
      
    #FILL_JOINT_STATE
    def fillCartesianPosition(self,message):
        #print(message)
        cartesianpos = message
        self.jointStateValues['cartesianPosition'] = [cartesianpos['x'], cartesianpos['y'], cartesianpos['z'] ,cartesianpos['a'], cartesianpos['e'], cartesianpos['r']]
     
    #METHODS FOR USING VALUES GIVEN BY E.DO
    def listenValues(self):
        self.listenCartesianPosition()
        self.listenJointState()
        if self.verbose: print('Subscribed')
        
    #GET_JOINT_STATE
    def getJointState(self):
        return [round(float(i),2) for i in self.jointStateValues]
   
    #FILL_MOVEMENT_ACK
    def fillMovementAck(self,message):
        #print(message)
        movement_ack = message
        self.jointStateValues['movementFeedback'] = movement_ack
    
    #UNLISTEN_MOVEMENT_ACK
    def unlistenJointState(self):
        self.MovementFeedback.unsubscribe()
        if self.verbose : print('Movement Feedback Unsubscribed')
    #UNLISTEN_CARTESIAN_POSE
    def unlistenCartesianPosition(self):
        self.CartesianPosition.unsubscribe()
        if self.verbose : print('Cartesian Position Unsubscribed')

# Class developed to extend edo class for euducational purposes
# 
# Put here all specific functions with data or demo purposes
class eduedo(edo):
    def __init__(self, host = '192.168.12.1', port = 9090, axes = 7):
        edo.__init__(self, host, port, axes )
        self.initMyedo()
        
        
    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper) [wrapper]
    def initMyedo(self):
        self.init7Axes()
        self.listenValues()
        time.sleep(1)
        print('e.DO is ready to have fun!')

    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper) [wrapper]
    def initMyedoVirtual(self):
        self.init7Axes()
        self.listenValues()
        time.sleep(1)
        self.disengageStd()
        time.sleep(2)
        self.calibAxes()
        self.moveJoints()
        self.gripperClose()
        time.sleep(1)
        print('e.DO is ready to have fun!')
        
    #DEMO of e.DO moving
    def rodeo(self):
        rodeoMovements = [[90, 50, -90, 0, 40, -90, 0], [-45, -50, 90, 0, -40, 45, 0], [60, 30, 95, 0, -45, 0, 80], [-30, 90, -40, -90, -90, 0, 0], [60, 40, -40, 90, -90, 0, 0], [0, -40, 80, 0, 80, 0, 80]]
        for move in rodeoMovements:
            self.command = {**self.commandTemplate}    
            self.command['move_command'] = MoveCommand.EXE_MOVE.value
            self.command['move_type'] = MoveType.JOINT.value
            self.command['delay'] = Delay.FLY.value
            self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
            self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
            self.command['target']['joints_data'] = move
            
            self.MovementCommand.publish(roslibpy.Message(self.command))
            
        if self.verbose: print('Rodeo demo completed')
        
    # Gripper movements
    def gripperOpen(self):
        self.moveGripper(j7=80)
        if self.verbose: print('Gripper opened')
    def gripperClose(self):
        self.moveGripper(j7=0)
        if self.verbose: print('Gripper closed')
        
    def gripperOpenCart(self):
        print('Gripper open Cartesian not yet supported')
    def gripperCloseCart(self):
        print('Gripper close Cartesian not yet supported')
        
    def setGripperOpen(self,j7):
        self.moveGripper(j7)
        if self.verbose: print('Gripper opened')
    def setGripperOpenCart(self,j7):
        print('Gripper open Cartesian not yet supported')
        
    def getMyedoState(self):
        print('Joints:   ',self.getJoints(),' [deg]')
        print('Cartesian:',self.getCartesianFull())
        print('Gripper:  ',self.getGripper(),' [mm]')
        
