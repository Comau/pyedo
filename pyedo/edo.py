## pyedo
##
## SP
## @file SDK_python.py
## @author  Comau
## @version 1.0
## @date 15.01.2020
## 
## @section LICENSE
##    This  material  is the exclusive property of Comau S.p.A.  and must be
##    returned   to   Comau   S.p.A.,   Robotics  Division,  Software  Group
##    immediately   upon   request.    This  material  and  the  information
##    illustrated or contained herein may not be used, reproduced, stored in
##    a retrieval system, or transmitted in whole or in part in  any  way  -
##    electronic, mechanical, photocopying, recording, or otherwise, without
##    the prior written consent of Comau S.p.A..
## 
##                      All Rights Reserved
##                      Copyright (C)  2020
##                          Comau S.p.A.
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
    EXE_JOGMOVE = 74 # 'J'
    EXE_JOGSTOP = 83 # 'S'
    EXE_MOVE = 77 # 'M'
    PAUSE = 80 # 'P' Pause movement execution
    RESUME = 82 # 'R' Resume movement execution
    CANCEL_MOVE = 67 # 'C' Cancel movement execution and empty robot queue

@unique
class MoveType(Enum):
    JOINT = 74 # 'J'
    LINEAR = 76 # 'L',
    CIRCULAR = 67 # 'C'

@unique
class DataType(Enum):
    E_MOVE_POINT_JOINT = 74 # 'J'
    E_MOVE_POINT_POSITION = 80 # 'P'
    E_MOVE_POINT_XTND_POS = 88 #'X'

@unique
class MaskType(Enum):
    JOINT_MASK6 = 63 # Joint Mask for 6 joints
    JOINT_MASK7 = 127 # Joint Mask for 6 joints + gripper
    JOINT_MASK7_EXT = 64 # Joint Mask for 7th axis

@unique
class Delay(Enum):
    FLY = 255
    ZERO_DELAY = 0

class edo(object):
    def __init__(self,host = '192.168.12.1',port = 9090):
        ''' Constructor for this class. '''
        self.port = port
        self.host = host
        # Standard Command Template
        self.command = {}
        self.command_template = {
                 "move_command": 0,
                 "move_type": 0,
                 "ovr": 0,
                 "delay": 0,
                 "remote_tool": 0,
                 "cartesian_linear_speed": 0.0,
                 "target": {
                     "data_type": 0,
                     "cartesian_data": {"x": 0.0, "y": 0.0, "z": 0.0, "a": 0.0, "e": 0.0, "r": 0.0, "config_flags": ''},
                     "joints_mask": 0,
                     "joints_data": [0]},
                 "via": {
                     "data_type": 0,
                     "cartesian_data": {"x": 0.0, "y": 0.0, "z": 0.0, "a": 0.0, "e": 0.0, "r": 0.0, "config_flags": ''},
                     "joints_mask": 0,
                     "joints_data": [0]},
                 "tool": {"x": 0.0, "y": 0.0, "z": 0.0, "a": 0.0, "e": 0.0, "r": 0.0},
                 "frame": {"x": 0.0, "y": 0.0, "z": 0.0, "a": 0.0, "e": 0.0, "r": 0.0}}
        # Methods Properties
        self.Verbose = True
        # Joints
        self.jointStateValues = dict()
        # KeyButton for JOG-Messages
        self.Mode = 0
        #self.JointArray = array('f',[0,0,0,0,0,0,0])
        self.JointArray = [0,0,0,0,0,0,0]
        self.JointJSON = json.dumps(self.JointArray)
        self.KeyButton = 0
        self.Mode = MoveType.JOINT.value
        self.Joint_Selected = 1
        client = roslibpy.Ros(self.host, self.port) # Wi-Fi
        client.run()
        if client.is_connected == False:
            print('WebSocket is busy, disconnect the e.DO App')
        else:
            print('Connected with e.DO')
            self.JointInit = roslibpy.Topic(client, '/bridge_init', 'edo_core_msgs/JointInit')
            self.JointReset = roslibpy.Topic(client, '/bridge_jnt_reset', 'edo_core_msgs/JointReset')
            self.JointCalibration = roslibpy.Topic(client, '/bridge_jnt_calib', 'edo_core_msgs/JointCalibration')
            self.MovementCommand = roslibpy.Topic(client, '/bridge_move', 'edo_core_msgs/MovementCommand')
            self.JogCommand = roslibpy.Topic(client, '/bridge_jog', 'edo_core_msgs/MovementCommand',queue_size = 1)
            self.MovementFeedback = roslibpy.Topic(client, '/machine_movement_ack', 'edo_core_msgs/MovementFeedback',throttle_rate=200)
            self.CartesianPosition = roslibpy.Topic(client, '/cartesian_pose', 'edo_core_msgs/CartesianPose',throttle_rate=200)
            self.JointState = roslibpy.Topic(client, '/usb_jnt_state', 'edo_core_msgs/JointStateArray',throttle_rate=200)
            
    
    # VERBOSITY MODE ON
    def verbose_on(self):
        self.Verbose = True
        print('Verbose mode ON')
    
    # VERBOSITY MODE OFF
    def verbose_off(self):
        self.Verbose = False
        print('Verbose mode OFF')
    
    # INITIALIZATION of the Robot with the Gripper
    def init_7Axes(self):
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK7.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command))
        if self.Verbose : print('e.DO with Gripper has been initialized')
    
    # INITIALIZATION of the Robot with 6 Joints
    def init_6Axes(self):
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK6.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command))
        if self.Verbose : print('e.DO with 6 Joints has been initialized')
    
    # STANDARD DISENGAGE of the Robot    
    def disengage_std(self):
        self.command = {
            "joints_mask": MaskType.JOINT_MASK7.value,
            "disengage_steps": 2000,
            "disengage_offset": 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Standard Disengage')
    
    # SINUSOIDAL DISENGAGE of the Robot 
    def disengage_sin(self):
        self.command = {
            "joints_mask": MaskType.JOINT_MASK7.value,
            "disengage_steps": 1,
            "disengage_offset": 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Sinusoidal Disengage')
    
    # SAFE DISENGAGE of the Robot without the any disengage movement
    def disengage_safe(self):
        self.command = {
            "joints_mask": MaskType.JOINT_MASK7.value,
            "disengage_steps": 1,
            "disengage_offset": 0.0
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Safe Disengage')
    
    #CALIBRATION of all the axes the Robot (Be sure that the notches are aligned)
    def calib_axes(self):
        self.command = {"joints_mask": MaskType.JOINT_MASK7.value}
        self.JointCalibration.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Calibration Axes')
    
    #JOINT MOVE
    def move_joint(self,ovr=30, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6, j7]
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Joint')
    
    #GRIPPER MOVE
    def move_gripper(self,ovr=30, j7=0.0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0, 0, 0, 0, 0, 0, j7]
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Gripper')
    
    #JOINT JOG MOVE  
    def jog_joint(self,ovr=100, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['ovr'] = ovr
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6, j7]
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Jog Joint')
    
    #STOP JOG
    def jog_stop(self):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_JOGSTOP.value
        self.command['delay'] = Delay.FLY.value
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Jog Stop')
    
    #CARTESIAN JOG MOVE  
    def jog_cartesian(self,ovr=100, x=0, y=0, z=0, a=0, e=0, r=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['ovr'] = ovr
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Jog Cartesian')
    
    #CARTESIAN MOVE  
    def move_cartesian(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Cartesian')
    
    #EXTENDED MOVE  
    def move_cartesianX(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Cartesian Extended')
    
    #CIRCULAR MOVE
    def move_circular(self,ovr=30, x1=447, y1=0, z1=180, a1=0, e1=180, r1=0, x2=0, y2=432, z2=15, a2=180, e2=0, r2=180):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Circular')
    
    #CIRCULAR MOVE EXTENDED
    def move_circularX(self,ovr=30, x1=480, y1=23, z1=640, a1=-12, e1=108, r1=0, j71=0, x2=480, y2=23, z2=640, a2=-12, e2=108, r2=0, j72=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j71]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j72]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Circular Extended')
    
    #CANCEL MOVE
    def move_cancel(self):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.CANCEL_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Cancel')

    #COOPERATIVE MOVE
    def move_cooperative(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Cooperative')
        
    #COOPERATIVE EXTENDED MOVE
    def move_cooperativeX(self,ovr=30, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        self.command = {**self.command_template}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['ovr'] = ovr
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        if ovr>100:
            self.command['ovr'] = 100
            print('Max override set equal to 100%')
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.Verbose : print('Move Cooperative Extended')
        
    #LISTEN_JOINT_STATE
    def listen_JointState(self):
        #self.JointState.subscribe(lambda message: print('messaggio = ' + message['data']))
        self.JointState.subscribe(self.fill_jointStateValues)
      
    #FILL_JOINT_STATE
    def fill_jointStateValues(self,message):
        self.joints = message['joints'];
        self.jointStatePosition=[]
        self.jointStateVelocity=[]
        self.jointStateCurrent=[]
        for vi_Idx in range(len(self.joints)):
             self.jointStatePosition.append(self.joints[vi_Idx]['position'])
             self.jointStateVelocity.append(self.joints[vi_Idx]['velocity'])
             self.jointStateCurrent.append(self.joints[vi_Idx]['current'])
        
        self.jointStateValues['position'] = self.jointStatePosition
        self.jointStateValues['velocity'] = self.jointStateVelocity
        self.jointStateValues['currrent'] = self.jointStateCurrent
        
    #LISTEN_CARTESIAN_POSE
    def listen_CartesianPosition(self):
        self.CartesianPosition.subscribe(self.fill_CartesianPosition)
        
    #FILL_JOINT_STATE
    def fill_CartesianPosition(self,message):
        #print(message)
        cartesianpos = message
        self.jointStateValues['cartesianPosition'] = [cartesianpos['x'], cartesianpos['y'], cartesianpos['z'] ,cartesianpos['a'], cartesianpos['e'], cartesianpos['r']]
        
    #GET_JOINT_STATE
    def get_JointState(self):
        return self.jointStateValues
        
    #UNLISTEN_CARTESIAN_POSE
    def unlisten_CartesianPosition(self):
        self.CartesianPosition.unsubscribe()
        if self.Verbose : print('Cartesian Position Unsubscribed')
        
    #UNLISTEN_JOINT_STATE
    def unlisten_JointState(self):
        self.JointState.unsubscribe()
        if self.Verbose : print('Joint State Unsubscribed')
        
    #LISTEN_MOVEMENT_ACK
    def listen_MovementAck(self):
        self.MovementFeedback.subscribe(self.fill_MovementAck)
        
    #FILL_MOVEMENT_ACK
    def fill_MovementAck(self,message):
        #print(message)
        movement_ack = message
        self.jointStateValues['movementFeedback'] = movement_ack
    
    #UNLISTEN_MOVEMENT_ACK
    def unlisten_JointState(self):
        self.MovementFeedback.unsubscribe()
        if self.Verbose : print('Movement Feedback Unsubscribed')
    