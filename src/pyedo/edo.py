## pyedo
##
## SP
## @file SDK_python.py
## @author  Comau
## @version 0.6
## @date 09.11.2021
## 

from platform import machine
import time
import roslibpy
from array import *
from pynput import keyboard
import queue 
import threading 
import json
from json import encoder
from enum import Enum, Flag, unique


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
        self.initialized = False
        self.disengaged = False
        self.interrupt = False
        # Standard initialization to zero
        self.jointStateValues = {'jointPosition': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'current': [0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0], 'cartesianPosition': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        self.movementAckValue = {'movementFeedback': {'data': 0, 'type': 0}}
        self.stepByStep = True # if True wait the end of current e.DO movement before moving to next command

        #CONNECTION TO e.DO
        self.client = roslibpy.Ros(self.host, self.port) # Wi-Fi
        self.client.run()
        if self.client.is_connected == False:
            print('WebSocket is busy, please disconnect the e.DO App')
        else:
            self.JointInit = roslibpy.Topic(self.client, '/bridge_init', 'edo_core_msgs/JointInit')
            self.JointReset = roslibpy.Topic(self.client, '/bridge_jnt_reset', 'edo_core_msgs/JointReset')
            self.JointCalibration = roslibpy.Topic(self.client, '/bridge_jnt_calib', 'edo_core_msgs/JointCalibration')
            self.JointState = roslibpy.Topic(self.client, '/usb_jnt_state', 'edo_core_msgs/JointStateArray',throttle_rate=200)
            self.MovementCommand = roslibpy.Topic(self.client, '/bridge_move', 'edo_core_msgs/MovementCommand')
            self.MovementFeedback = roslibpy.Topic(self.client, '/machine_movement_ack', 'edo_core_msgs/MovementFeedback',queue_size = 1)
            self.JogCommand = roslibpy.Topic(self.client, '/bridge_jog', 'edo_core_msgs/MovementCommand',queue_size = 1)
            self.MachineState = roslibpy.Topic(self.client, '/machine_state', 'edo_core_msgs/MachineState')
            self.CartesianPosition = roslibpy.Topic(self.client, '/cartesian_pose', 'edo_core_msgs/CartesianPose')
            print('Connected with e.DO')
    
    # Listening from keyboard
        listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        listener.start()

    def on_press(self,key):
        try:
            if key.char == 'k':
                self.interrupt = True
                self.moveCancel()
                if not self.stepByStep:
                    self.interrupt = False
        except AttributeError:
            pass

    def on_release(self,key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    # waiting for the aknowledgment to finish the move
    def waitAcknowledgment(self):
        '''Wait for the end-of-movement acknowledge. The flag stepByStep must be true. The movement can be interrupted by pressing the 'x' key.

        :return None
        '''
        time.sleep(1)
        if self.stepByStep:
            while self.movementAckValue['movementFeedback']['type'] != 2:
                if self.movementAckValue['movementFeedback']['type'] == -1:
                    raise ValueError('ERROR. Robot might be in singularity position. Move in the Joint Space to exit singularity.')
                if self.interrupt:
                    self.interrupt = False
                    self.moveCancel()
                    break


    # VERBOSITY MODE ON
    def verboseOn(self):
        self.verbose = True
        print('Verbose mode ON')
    
    # VERBOSITY MODE OFF
    def verboseOff(self):
        self.verbose = False
        print('Verbose mode OFF')

    # stepByStep MODE ON
    def stepByStepOn(self):
        self.stepByStep = True
        if self.verbose: print('stepByStep mode ON')
    
    # stepByStep MODE OFF
    def stepByStepOff(self):
        self.stepByStep = False
        if self.verbose: print('stepByStep mode OFF')
    
    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper)
    def init7Axes(self):
        '''Initialize the robot with 7 axes (e.DO 6 axes plus the gripper).

        :return None
        '''
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK7.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command)) 
        self.initialized = True
        print('e.DO with Gripper has been initialized')
    
    # INITIALIZATION of the Robot with 6 Joints
    def init6Axes(self):
        '''Initialize the robot with 6 axes.
        
        :return None
        '''
        self.command = {
            "mode": 0,
            "joints_mask": MaskType.JOINT_MASK6.value,
            "reduction_factor": 0.0
        }
        self.JointInit.publish(roslibpy.Message(self.command)) 
        self.initialized = True
        print('e.DO with 6 Joints has been initialized')

        
    # STANDARD DISENGAGE of the Robot    
    def disengageStd(self):
        '''Disengage the robot with the standard movement (RECOMMENDED).

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 2000,
            'disengage_offset': 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        self.disengaged = True
        if self.verbose: print('Standard Disengage')
    
    # SINUSOIDAL DISENGAGE of the Robot 
    def disengageSin(self):
        '''Disegage the robot with a sinusoidal movement (MANDATORY TO HAVE THE JOINTS FAR AWAY FROM THE STROKE END).

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 1,
            'disengage_offset': 3.5
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        self.disengaged = True
        if self.verbose : print('Sinusoidal Disengage')
    
    # SAFE DISENGAGE of the Robot without the any disengage movement
    def disengageSafe(self):
        '''Release only the brakes.

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command = {
            'joints_mask': MaskType.JOINT_MASK7.value,
            'disengage_steps': 1,
            'disengage_offset': 0.0
        }
        self.JointReset.publish(roslibpy.Message(self.command))
        self.disengaged = True
        if self.verbose: print('Safe Disengage')
    
    #CALIBRATION of all the axes the Robot (Be sure that the notches are aligned)
    def calibAxes(self):
        '''Calibrate all the robot axes (MANDATORY TO HAVE THE JOINTS ALLIGNED WITH THE NOTCHES (Home Position)).
         The calibration is possible only after the initizialization(init method) and the disengage of the brakes(disengage method).

         :return None
        '''
        if self.initialized and self.disengaged:
            self.command = {**self.commandTemplate}
            self.command = {"joints_mask": MaskType.JOINT_MASK7.value}
            self.JointCalibration.publish(roslibpy.Message(self.command))
            if self.verbose: print('Calibration Axes')
        else:
            if (not self.disengaged) and self.initialized:
                raise ValueError('Error: Not disengaged')
            if (not self.initialized) and self.disengaged:
                raise ValueError('Error: Not initialized')
            if (not self.disengaged) and (not self.initialized):
                raise ValueError('Error: Not disengaged and not initialized')
    
    #SET SPEED (Default = 100%, values bigger than 100 not accepted)
    def setSpeed(self, ovr = 100):
        '''Set the percentage speed of all joints.

        :param ovr: between 0 and 100
        :type ovr: float

        :return None
        '''
        if ovr > 100: 
            print('ATTENTION. Max speed = 100%.')
            ovr = 100
        elif ovr < 0:
            print('ATTENTION. Min speed = 0%.')
            ovr = 0     
        self.commandTemplate['ovr'] = ovr


    #Move e.DO to vertical position (calibration)
    def moveToHome(self):
        ''' Send a command with target in zero for all the joints and the gripper. Move eDo to the home position.

        :return None
        '''
        self.command = {**self.commandTemplate}    
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0, 0, 0, 0, 0, 0, 0]
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        self.moveGripper(0)

        self.waitAcknowledgment()
        if self.verbose: print('Move to Home Position')


    #Move e.DO to the waiting position (cartesian movements)
    def moveToWaitingPos(self):
        ''' Send a command with target waiting position and target zero for the gripper.

        :return None
        '''
        self.command = {**self.commandTemplate}    
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0, 40, 50, 0, 90, 0, 0]
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        self.moveGripper(0)

        self.waitAcknowledgment()
        if self.verbose: print('Move to Waiting Position')


    #JOINT MOVE
    def moveJoints(self, j1 = 0.0, j2 = 0.0, j3 = 0.0, j4 = 0.0, j5 = 0.0, j6 = 0.0):
        '''Move the joints to a target position expressed in degrees.

        :param j1: target 1st joint [deg], range [-178,178]
        :type j1: float

        :param j2: target 2nd joint [deg], range [-99,99]
        :type j2: float

        :param j3: target 3rd joint [deg], range [-99,99]
        :type j3: float

        :param j4: target 4th joint [deg], range [-178,178]
        :type j4: float

        :param j5: target 5th joint [deg], range [-103,103]
        :type j5: float

        :param j6: target 6th joint [deg], range [-178,178]
        :type j6: float

        :return None
        '''
        if (j1 < -178 or j1 > 178):
            raise ValueError('ERROR. J1 value not acceptable.')
        if j2 < -99 or j2 > 99:
            raise ValueError('ERROR. J2 value not acceptable.')
        if j3 < -99 or j3 > 99:
            raise ValueError('ERROR. J3 value not acceptable.')
        if j4 < -178 or j4 > 178:
            raise ValueError('ERROR. J4 value not acceptable.')
        if j5 < -103 or j5 > 103:
            raise ValueError('ERROR. J5 value not acceptable.')
        if j6 < -178 or j6 > 178:
            raise ValueError('ERROR. J6 value not acceptable.')
        else:
            self.command = {**self.commandTemplate}
            self.command['move_command'] = MoveCommand.EXE_MOVE.value
            self.command['move_type'] = MoveType.JOINT.value
            self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
            self.command['target']['joints_mask'] = MaskType.JOINT_MASK6.value
            self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6]
            
            self.MovementCommand.publish(roslibpy.Message(self.command))
            
            self.waitAcknowledgment()
            if self.verbose: print('Move Joints')


    def moveSingleJoint(self, num, value):
        '''Send the target position for a chosen joint.
        
        :param num: selected joint
        :type num: int

        :param value: target position for the joint num. Accepted values smaller (in module) than: 178,99,99,178,103,178
        :type value: float

        :return None
        '''
        if num not in range(1,7): 
            raise ValueError('ERROR. Choose a joint number between 1 and 6.')

        if (num == 1 or num == 4 or num == 6) and (value < -178 or value > 178):
            raise ValueError('ERROR. Joint value not acceptable.')

        if (num == 2 or num == 3) and (value < -99 or value > 99):
            raise ValueError('ERROR. Joint value not acceptable.')

        if num == 5 and (value < -103 or value > 103):
            raise ValueError('ERROR. Joint value not acceptable.')
        else:
            self.command = {**self.commandTemplate}
            self.command['move_command'] = MoveCommand.EXE_MOVE.value
            self.command['move_type'] = MoveType.JOINT.value
            self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
            self.command['target']['joints_mask'] = pow(2, num - 1)
            self.command['target']['joints_data'] = [value, value, value, value, value, value]

            
            self.MovementCommand.publish(roslibpy.Message(self.command))

            self.waitAcknowledgment()
            if self.verbose: print('Move Single Joint')
            

    #CARTESIAN MOVE  
    def moveCartesian(self, x = 370, y = 0, z = 210, a = 0, e = 180, r = 0):
        '''Move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r).

        :param x: target on x [mm] - Default 370
        :type x: float

        :param y: target on y [mm] - Default 0
        :type y: float

        :param z: target on z [mm] - Default 210
        :type z: float

        :param a: target a [deg] - Default 0
        :type a: float

        :param e: target e [deg] - Default 180
        :type e: float

        :param r: target r [deg] - Default 0
        :type r: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {'x': x, 'y': y, 'z': z, 'a': a, 'e': e, 'r': r, 'config_flags': ''}

        self.MovementCommand.publish(roslibpy.Message(self.command))

        self.waitAcknowledgment()
        if self.verbose: print('Move Cartesian')    


    #GRIPPER MOVE
    def moveGripper(self,j7=0.0):
        ''' Set the desired width of the gripper.

        :param j7: width [mm] - between 0 and 80
        :type j7: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        # Stroke end check
        if j7 < 0: 
            print('ATTENTION. Min gripper width = 0 mm.')
            print('Gripper will close to 0 mm.')
            j7 = 0
        elif j7 > 80:
            print('ATTENTION. Max gripper width = 80 mm.')  
            print('Gripper will open to 80 mm.')
            j7 = 80
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0, 0, 0, 0, 0, 0, j7]
                
        self.MovementCommand.publish(roslibpy.Message(self.command))

        self.waitAcknowledgment()
        if self.verbose : print('Move Gripper')
        
    def moveGripperCart(self, j7=40):
        '''Use the gripper in the cartesian space.

        :param j7: target gripper value - Default 40
        :type j7: float

        :return None
        '''
        cart = self.getCartesianFull()
        self.moveCartesianX(cart[0], cart[1], cart[2], cart[3], cart[4], cart[5], j7)

        if self.verbose: print('Move Gripper Cartesian')    
        
    def gripperWidth(self, toInteger = True): # if FALSE = return gripper width in float with 14 decimal values
        ''' Return the current gripper width (between 0 and 80). If toInteger set to False returns a float with 14 decimal values.

        :param toInteger: OPTIONAL - default True
        :type toInteger: boolean

        :returns width [mm]
        :rtype: int by default, float if toInteger set to False
        '''
        self.listenValues()
        time.sleep(1)
        if toInteger:
            return int(self.getGripper()) 
        else:
            return self.getGripper()
    

    
    #JOINT JOG MOVE  
    def jogJoint(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, j7=0.0):
        '''Move the joints, once per time for a delta, in a positive or negative direction.

        :param j1: delta 1st joint [deg]
        :type j1: float

        :param j2: delta 2nd joint [deg]
        :type j12: float
        
        :param j3: delta 3rd joint [deg]
        :type j3: float
        
        :param j4: delta 4th joint [deg]
        :type j4: float
        
        :param j5: delta 5th joint [deg]
        :type j5: float
        
        :param j6: delta 6th joint [deg]
        :type j6: float
        
        :param j7: delta gripper [mm]
        :type j7: float

        :return None
        '''
        # Check if out of range:
        if (j1 > 0 and (self.getJoints()[0] + j1) >= 178) or (j1 < 0 and (self.getJoints()[0] + j1) <= -178):
            print('Attention: joint 1 at its limit.')
            j1 = 0 # not reaching the exact limit
        if (j2 > 0 and (self.getJoints()[1] + j2) >= 99) or (j2 < 0 and (self.getJoints()[1] + j2) <= -99): 
            print('Attention: joint 2 at its limit.')
            j2 = 0
        if (j3 > 0 and (self.getJoints()[2] + j3) >= 99) or (j3 < 0 and (self.getJoints()[2] + j3) <= -99):
            print('Attention: joint 3 at its limit.')
            j3 = 0
        if (j4 > 0 and (self.getJoints()[3] + j4) >= 178) or (j4 < 0 and (self.getJoints()[3] + j4) <= -178):
            print('Attention: joint 4 at its limit.')
            j4 = 0
        if (j5 > 0 and (self.getJoints()[4] + j5) >= 103) or (j5 < 0 and (self.getJoints()[4] + j5) <= -103):
            print('Attention: joint 5 at its limit.')
            j5 = 0
        if (j6 > 0 and (self.getJoints()[5] + j6) >= 178) or (j6 < 0 and (self.getJoints()[5] + j6) <= -178):
            print('Attention: joint 6 at its limit.')
            j6 = 0
        if (j7 > 0 and (self.getGripper() + j7) >= 80) or (j7 < 0 and (self.getGripper() + j7) <= 0):
            print('Attention: gripper limit reached.')
            j7 = 0

        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [j1, j2, j3, j4, j5, j6, j7]
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        
        if self.verbose : print('Jog Joint')
    

    #STOP JOG
    def jogStop(self):
        ''' Stop the jogging.

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGSTOP.value
        self.command['delay'] = Delay.FLY.value
        self.JogCommand.publish(roslibpy.Message(self.command))
        
        if self.verbose : print('Jog Stop')
    

    #CARTESIAN JOG MOVE  
    def jogCartesian(self, x=0, y=0, z=0, a=0, e=0, r=0):
        '''Move the joints in a cartesian position, one coordinate per time for a delta value.

        :param x: delta on x [mm]
        :type x: float

        :param y: delta on y [mm]
        :type y: float
        
        :param z: delta on z [mm]
        :type z: float

        :param a: delta on azimuth [deg]
        :type a: float

        :param e: delta on elevation [deg]
        :type e: float
        
        :param r: delta on roll [deg]
        :type r: float

        :return None
        '''
        if self.getJoints() == [0,0,0,0,0,0]:
            raise ValueError('ERROR. Cannot move cartesian from Home position. Move in the Joint Space to exit singularity. (Suggested: moveToWaitingPos()')

        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_JOGMOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['delay'] = Delay.FLY.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        self.JogCommand.publish(roslibpy.Message(self.command))
        
        if self.verbose : print('Jog Cartesian')
     

    #EXTENDED MOVE  
    def moveCartesianX(self, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0):
        '''Move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) and the opening position of the gripper.

        :param x: target on x [mm] - Default 480
        :type x: float

        :param y: target on y [mm] - Default 23
        :type y: float

        :param z: target on z [mm] - Default 640
        :type z: float

        :param a: target a [deg] - Default -12
        :type a: float

        :param e: target e [deg] - Default 108
        :type e: float

        :param r: target r [deg] - Default 0
        :type r: float

        :param j7: gripper width [mm] - Default 0
        :type j7: float

        :return None
        '''
        if self.getJoints() == [0,0,0,0,0,0]:
            raise ValueError('ERROR. Cannot move cartesian from Home position. Move in the Joint Space to exit singularity. (Suggested: moveToWaitingPos()')
        
        if j7 > 80:
            print('Attention: the gripper will open to 80mm')
            j7 = 80
        if j7 < 0:
            print('Attention: the gripper will close to 0')
            j7 = 0
        
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        self.waitAcknowledgment()
        self.waitAcknowledgment()  # one for the cartesian movement, the other for the gripper movement
        if self.verbose : print('Move Cartesian Extended')
    

    #CIRCULAR MOVE
    def moveCircular(self, x1=447, y1=0, z1=180, a1=0, e1=180, r1=0, x2=0, y2=432, z2=15, a2=180, e2=0, r2=180):
        '''Move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) passing through another cartesian_position 
          indicated through the pose in (x2, y2, z2, a2, e2, r2), creating a circular path.

        :param x1: target on x [mm] - Default 447
        :type x1: float

        :param y1: target on y [mm] - Default 0
        :type y1: float

        :param z1: target on z [mm] - Default 180
        :type z1: float

        :param a1: target a [deg] - Default 0
        :type a1: float

        :param e1: target e [deg] - Default 180
        :type e1: float

        :param r1: target r [deg] - Default 0
        :type r1: float

        :param x2: intermediate target on x [mm] - Default 0
        :type x2: float

        :param y2: intermediate target on y [mm] - Default 432
        :type y2: float

        :param z2:intermediate target on z [mm] - Default 15
        :type z2: float

        :param a2: intermediate target a [deg] - Default 180
        :type a2: float

        :param e2: intermediate target e [deg] - Default 0
        :type e2: float

        :param r2: intermediate target r [deg] - Default 180
        :type r2: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        self.waitAcknowledgment()
        if self.verbose : print('Move Circular')
    

    #CIRCULAR MOVE EXTENDED
    def moveCircularX(self, x1=480, y1=23, z1=640, a1=-12, e1=108, r1=0, j71=0, x2=480, y2=23, z2=640, a2=-12, e2=108, r2=0, j72=0):
        '''Move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) and the opening position of the gripper in (j71) passing through another cartesian_position 
          indicated through the pose in (x2, y2, z2, a2, e2, r2) and the opening position of the gripper in (j72), creating a circular path.

        :param x1: target on x [mm] - Default 480
        :type x1: float

        :param y1: target on y [mm] - Default 23
        :type y1: float

        :param z1: target on z [mm] - Default 640
        :type z1: float

        :param a1: target a [deg] - Default -12
        :type a1: float

        :param e1: target e [deg] - Default 108
        :type e1: float

        :param r1: target r [deg] - Default 0
        :type r1: float

        :param j71: gripper width [mm] - Default 0
        :type j71: float

        :param x2: intermediate target on x [mm] - Default 0
        :type x2: float

        :param y2: intermediate target on y [mm] - Default 432
        :type y2: float

        :param z2:intermediate target on z [mm] - Default 15
        :type z2: float

        :param a2: intermediate target a [deg] - Default 180
        :type a2: float

        :param e2: intermediate target e [deg] - Default 0
        :type e2: float

        :param r2: intermediate target r [deg] - Default 180
        :type r2: float

        :param j72: intermediate gripper width [mm] - Default 0
        :type j72: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.CIRCULAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j71]
        self.command['target']['cartesian_data'] = {"x": x1, "y": y1, "z": z1, "a": a1, "e": e1, "r": r1, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j72]
        self.command['via']['cartesian_data'] = {"x": x2, "y": y2, "z": z2, "a": a2, "e": e2, "r": r2, "config_flags": ''}
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Circular Extended')
    

    #CANCEL MOVE
    def moveCancel(self):
        '''Cancel the buffer of the moves, helpful when some erros occur in generating a path trajectory which is not allowed.

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.CANCEL_MOVE.value
        self.command['move_type'] = MoveType.JOINT.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_JOINT.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cancel')


    #COOPERATIVE MOVE
    def moveCooperative(self,x=480, y=23, z=640, a=-12, e=108, r=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        '''Used in cooperative tasks. The first set of coordinates represents the robot cartesian frame,
           the second set of coordinates is the tool frame, common to all the arms involved in the manipulation.
        
        :param x: target x arm
        :type x: float

        :param y: target y arm
        :type y: float

        :param z: target z arm
        :type z: float

        :param a: target a arm
        :type a: float

        :param e: target e arm
        :type e: float

        :param r: target r arm
        :type r: float

        :param xf: target x object frame
        :type xf: float

        :param yf: target y object frame
        :type yf: float

        :param zf: target z object frame
        :type zf: float

        :param af: target a object frame
        :type af: float

        :param ef: target e object frame
        :type ef: float

        :param rf: target r object frame
        :type rf: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_POSITION.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7.value
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cooperative')
        

    #COOPERATIVE EXTENDED MOVE
    def moveCooperativeX(self, x=480, y=23, z=640, a=-12, e=108, r=0, j7=0, xf=480, yf=23, zf=640, af=-12, ef=108, rf=0):
        '''Used in cooperative tasks. The first set of coordinates represents the robot cartesian frame, j7 is the gripper width [mm],
         the second set of coordinates is the tool frame, common to all the arms involved in the manipulation.

        :param x: target x arm
        :type x: float

        :param y: target y arm
        :type y: float

        :param z: target z arm
        :type z: float

        :param a: target a arm
        :type a: float

        :param e: target e arm
        :type e: float

        :param r: target r arm
        :type r: float

        :param j7: target x arm
        :type j7: float

        :param xf: target x object frame
        :type xf: float

        :param yf: target y object frame
        :type yf: float

        :param zf: target z object frame
        :type zf: float

        :param af: target a object frame
        :type af: float

        :param ef: target e object frame
        :type ef: float

        :param rf: target r object frame
        :type rf: float

        :return None
        '''
        self.command = {**self.commandTemplate}
        self.command['move_command'] = MoveCommand.EXE_MOVE.value
        self.command['move_type'] = MoveType.LINEAR.value
        self.command['target']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['target']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['target']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['target']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['via']['data_type'] = DataType.E_MOVE_POINT_XTND_POS.value
        self.command['via']['joints_mask'] = MaskType.JOINT_MASK7_EXT.value
        self.command['via']['joints_data'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7]
        self.command['via']['cartesian_data'] = {"x": x, "y": y, "z": z, "a": a, "e": e, "r": r, "config_flags": ''}
        self.command['frame'] = {"x": xf, "y": yf, "z": zf, "a": af, "e": ef, "r": rf}
        
        self.MovementCommand.publish(roslibpy.Message(self.command))
        if self.verbose : print('Move Cooperative Extended')
        

    #FILL VALUES
    def fillJointStateValues(self, message):
        '''Callback function that reads from the ROS topic the following information and feeds it to the Joint State dictionary:
                [Position 
                 Velocity
                 Current]

        :param message: the ROS message read from the topic /usb_jnt_state, containing the jointStateValues
        :message type: edo_core_msgs/JointStateArray

        :return None
        '''
        self.joints = message['joints']
        self.jointStatePosition = []
        self.jointStateVelocity = []
        self.jointStateCurrent  = []
        for vi_Idx in range(len(self.joints)):
            self.jointStatePosition.append(self.joints[vi_Idx]['position'])
            self.jointStateVelocity.append(self.joints[vi_Idx]['velocity'])
            self.jointStateCurrent.append(self.joints[vi_Idx]['current'])
        
        self.jointStateValues['jointPosition'] = self.jointStatePosition
        self.jointStateValues['velocity']      = self.jointStateVelocity
        self.jointStateValues['current']      = self.jointStateCurrent
    
    
    # Print Joint values [deg] rounded to the 2nd decimal
    def getJoints(self):
        '''Returns the joint position for each joint.
        
        :returns jointStatePosition [deg]
        :rtype: float rounded to the 2nd decimal
        '''
        return [round(float(i),2) for i in self.jointStatePosition[0:6]]


    # Print Cartesian X Y Z values [mm] rounded to the 2th decimal
    def getCartesian(self):  
        '''Returns the cartesian X Y Z values.

        :return cartesianPosition [mm]
        :rtype: float rounded to the 2nd decimal
        '''
        return [round(float(i),2) for i in self.jointStateValues['cartesianPosition'][0:3]]


    # Print Cartesian values rounded to the 2th decimal
    def getCartesianFull(self):
        '''Returns the cartesian values X Y Z, A E R.

        :return cartesianPosition XYZ [mm] AER [deg]
        :rtype: float rounded to the 2nd decimal
        '''
        return [round(float(i),2) for i in self.jointStateValues['cartesianPosition']]


    # Print gripper opening value in [mm] rounded to the 2nd decimal
    def getGripper(self): #TBD leggere da app_jnt_state -> AppStateArray
        '''Returns the gripper opening value.
        
        :return gripper width [mm]
        :rtype: float rounded to the 2nd decimal
        '''
        return round(float(self.jointStatePosition[6]),2)
        
    #LISTEN_MOVEMENT_ACK
    def listenMovementAck(self):
        '''Starts the subscription to the topic related to the movement aknowledge, which sends a feedback on the movement.

        :return None
        '''
        self.MovementFeedback.subscribe(self.fillMovementAck)
        if self.verbose : print('Subscribed to Movement Acknowledge')

    #LISTEN_CARTESIAN_POSE
    def listenCartesianPosition(self):
        '''Starts the subscription to the topic related to the cartesian position (x,y,z,a,e,r)

        :return None
        '''
        self.CartesianPosition.subscribe(self.fillCartesianPosition)
        if self.verbose : print('Subscribed to Cartesian Position')

        
    #LISTEN_JOINT_STATE
    def listenJointState(self):
        '''Starts the subscription to the topic related to these informations:
                    [Position 
                     Velocity
                     Current]
        
        :returns None
        '''
        #self.JointState.subscribe(lambda message: print('messaggio = ' + message['data']))
        self.JointState.subscribe(self.fillJointStateValues)
        if self.verbose : print('Subscribed to Joint Position')
      
    #FILL_JOINT_STATE
    def fillCartesianPosition(self,message):
        '''Callback that receives the cartesian position of the robot and feeds it to the jointStateValues variable.

        :param message: message read from the topic /cartesian_pose containing the cartesian position
        :type message: edo_core_msgs/CartesianPose

        :return None
        '''
        #print(message)
        cartesianpos = message
        self.jointStateValues['cartesianPosition'] = [cartesianpos['x'], cartesianpos['y'], cartesianpos['z'] ,cartesianpos['a'], cartesianpos['e'], cartesianpos['r']]
     
    #METHODS FOR USING VALUES GIVEN BY E.DO
    def listenValues(self):
        '''Calls listenCartesianPosition and listenJointState, hence starts the subscription to both cartesian and state variables.

        :return None
        '''
        self.listenCartesianPosition()
        self.listenJointState()
        self.listenMovementAck()
        if self.verbose: print('Subscribed')
    
    def unlistenValues(self):
        '''Calls unlistenCartesianPosition and unlistenJointState, hence stops the subscription to both cartesian and state variables.

        :return None
        '''
        self.unlistenCartesianPosition()
        self.unlistenJointState()
        self.unlistenMovementAck()
        if self.verbose: print('Unsubscribed')
        
    #GET_JOINT_STATE
    def getJointState(self):
        '''Returns the Joint State dictionary variables as lists of numbers:
                [Position 
                 Velocity 
                 Current 
                 cartesianPosition]
        
        :return jointStateValues list of lists
        :rtype: float rounded to the 2nd decimal
        '''
        jointStateValuesList = [[round(float(i),2) for i in self.jointStateValues['cartesianPosition']], [round(float(i),2) for i in self.jointStateValues['jointPosition']], [round(float(i),2) for i in self.jointStateValues['velocity']], [round(float(i),2) for i in self.jointStateValues['current']]]
        return jointStateValuesList


    #FILL_MOVEMENT_ACK
    def fillMovementAck(self,message):  
        '''Callback for receiving the movement feedback of the movementAckValue.

        :param message: message read from the topic /machine_movement_ack, containing the feedback
        :type message: edo_core_msgs/MovementFeedback

        :return None
        '''
        self.movementAckValue['movementFeedback'] = message # movement_ack
        
    #UNLISTEN_MOVEMENT_ACK
    
    def unlistenMovementAck(self):
        '''Stop the subscription to the topic related to the movement aknowledge, which sends a feedback on the movement.

        :return None
        '''
        self.MovementFeedback.unsubscribe()
        if self.verbose : print('Movement Feedback Unsubscribed')
        
    #UNLISTEN_JOINT_STATE
    def unlistenJointState(self):  # chiedere: ma il movement feedback, non l'ack?
        '''Stop the subscribing on the topic related to these informations:
                [Position 
                 Velocity
                 Current]

        :return None
        '''
        self.JointState.unsubscribe()
        time.sleep(0.5)
        if self.verbose : print('Unsubscribed to Joint Position')

    #UNLISTEN_CARTESIAN_POSE
    def unlistenCartesianPosition(self):
        '''Stop the subscribing on the topic related to the cartesian position (x,y,z,a,e,r).

        :return None
        '''
        self.CartesianPosition.unsubscribe()
        if self.verbose : print('Cartesian Position Unsubscribed')

    def disconnect(self):
        '''Disconnect the e.Do from the websocket. This is reversable: it is possible to connect again.

        :return None
        '''
        self.client.close() # disconnects from ROS
        while self.client.is_connected:
            time.sleep(0.5)
        print('Disconnected from ROS: can connect to the app.')

    def connect(self):
        '''Connects the e.Do to the websocket. Should be used when we want to riconnect after the disconnect() function was called.

        :return None
        '''
        self.client.connect()
        while not self.client.is_connected:
            time.sleep(0.5)
        print('Connected to ROS.')

    def safeShutdown(self):
        '''Safe shutdown for the robot. Stops the main event loop.

        :return None
        '''
        self.client.terminate()
        print('Disconnected and terminated: can turn off e.DO safely.')

    def unblock(self):
        '''Unblocks the e.Do in case it is not responding to commands.

        :return None
        '''
        self.moveCancel()
        self.disengageSafe()

# Class developed to extend edo class for euducational purposes
# 
# Put here all specific functions with data or demo purposes
class eduedo(edo):
    def __init__(self, host = '192.168.12.1', port = 9090, axes = 7):
        edo.__init__(self, host, port, axes )
        self.initMyedo()
        
        
    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper) [wrapper]
    def initMyedo(self):
        '''Initialize the robot with 7 axes (e.DO 6 axes plus the gripper) and subscribe to both cartesian and state variables.

        :return None
        '''
        self.init7Axes()
        self.listenValues()
        time.sleep(1)
        print('e.DO is ready to have fun!')

    # INITIALIZATION of the Robot with the Gripper (default = 6 axes + gripper) [wrapper]
    def initMyedoVirtual(self):
        '''- Initialize the robot with 7 axes (e.DO 6 axes plus the gripper).
           - Subscribe to both cartesian and state variables.
           - Calibrate the arm.

           :return None
        '''
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
        '''Demo of e.DO moving.

        :return None
        '''
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
        '''Open gripper with maximum width (80mm).

        :return None
        '''
        self.moveGripper(j7=80)
        if self.verbose: print('Gripper opened')
    def gripperClose(self):
        '''Close the gripper.

        :return None
        '''
        stepOff = False
        if self.stepByStep:
            self.stepByStepOff()
            stepOff = True
        self.moveGripper(j7=0)
        if self.verbose: print('Gripper closed')
        time.sleep(1)
        if stepOff:
            self.stepByStepOn()
        
    def gripperOpenCart(self):
        '''Not yet supported!
        '''
        print('Gripper open Cartesian not yet supported')
    def gripperCloseCart(self):
        '''Not yet supported!
        '''
        print('Gripper close Cartesian not yet supported')
        
    def setGripperOpen(self,j7):
        '''Open the gripper to a defined width.

        :param j7: width [mm] - from 0 to 80
        :type j7: float

        :return None
        '''
        self.moveGripper(j7)
        if self.verbose: print('Gripper opened')
    def setGripperOpenCart(self,j7):
        '''Not yet supported!
        '''
        print('Gripper open Cartesian not yet supported')
        
    def getMyedoState(self):
        '''Print Joint variables [deg], cartesian coordinates, gripper width [mm].

        :return None
        '''
        print('Joints:   ',self.getJoints(),' [deg]')
        print('Cartesian:',self.getCartesianFull())
        print('Gripper:  ',self.getGripper(),' [mm]')
        
    def disconnectMyedo(self):
        '''Disconnect the e.Do from the websocket
        
        :return None
        '''
        self.disconnect()
        if self.verbose: print('Edo disconnected')
        
    def reconnectMyedo(self):
        '''Reconnect the e.Do to the websocket. To be used after the disconnectMyedo() function, if necessary to reopen connection.
        
        :return None
            '''
        self.connect()
        if self.verbose: print('Edo reconnected')

    def safeShutdownMyedo(self):
        '''To be used before shutting down the robot.

        :return None
        '''
        # listener.stop()
        self.safeShutdown()
        if self.verbose : print('Safe to shutdown')


    def unblockMyedo(self):
        '''When e.Do is not responding to commands. Disengage, move to waiting position and open the gripper.

        :return None
        '''
        self.unblock()
        self.moveToWaitingPos()
        self.gripperOpen()