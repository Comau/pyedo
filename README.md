Pyedo
=======================================

This package contains the SDK to program e.DO robot with Python
<img src="https://edo.cloud/wp-content/uploads/2021/06/edo-6-axes.png" alt="eDo robot" width="100px"/> 


# Programming e.DO with Python

Quick installation
-----------
Install the Python version on your device, we suggest python >= 3.7.5 :

https://www.python.org/downloads/

There are two possibilities to install pyedo on your environment:

- Download the SDK from our site. Drag and drop the "pyedo" package into the python path site packages.

"C:\Users\YourAccount\AppData\Local\Programs\Python\PythonXX\Lib\site-packages\"

- Pyedo can be installed using pip:

        $ pip install pyedo

# How to start:
Create a new program, import the object from the pyedo library, create an instance of an eDO object in your program and now move e.DO with the available methods defined below.

```python
from pyedo import edo # import the object from the pyedo library
myedo = edo('10.42.0.49') # create an instance of an eDO object
```
You are now able to connect your device with e.DO via Python.

The addresses available for e.DO are: '10.42.0.49' for the Ethernet/LAN connection and '192.168.12.1' for the WiFi connection

# INIT
This method allows to initialize the e.DO robot, with or without gripper:
- init7Axes() :
Can initialize the robot with 7 axes (e.DO 6 axes plus the gripper)
- init6Axes() :
Can initialize the robot with 6 axes 

# DISENGAGE
This method allows to disengage the brakes:
- disengageStd() :
Can disengage the robot with the standard movement (RECOMMENDED)
- disengageSin() :
Can disegage the robot with a sinusoidal movement (MANDATORY TO HAVE THE JOINTS FAR AWAY FROM THE STROKE END)
- disengageSafe() :
Can release only the brakes
- unblock() :
When the e.Do is not responding to commands (not singularity).

# CALIB
This method allows to calibrate all the joints:
- calibAxes() :
Can calibrate all the robot axes (MANDATORY TO HAVE THE JOINTS ALLIGNED WITH THE NOTCHES (Home Position)) 
The calibration is possible only after the initizialization(init method) and the disengage of the brakes(disengage method)

# SET
This method sets the joints speed parameter:
- setSpeed(ovr) :
 Sets the speed, in percentage value, of all joints. The parameter of the function is a number between «0» and «100». The default value of ovr is «100».
 It prints a warning if the parameter passed is not in the accepted range.

# MOVE
This method allows to move the e.DO robot. The setting automatically waits for the movement to be over before giving the next command (can be deactivated with stepByStepOff). The pianification of the trajectory can be performed in different ways:
- moveJoints(j1, j2, j3, j4, j5, j6) :
Can move the joints to a joint_position indicated through the angles in degrees (j1, , j2, j3, j4, j5, j6) with a velocity percentage determined by the setSpeed() function
- moveSingleJoint(num, value) :
Can move a single joint, indicated in num, to a target position given by the value parameter
- moveCartesian(x, y, z, a, e, r) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) with a velocity percentage determined by the setSpeed() function
- moveCartesianX(x, y, z, a, e, r, j7) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) and the opening position in mm of the gripper  in (j7) with a velocity percentage determined by the setSpeed() function
- moveCircular(x1, y1, z1, a1, e1, r1, x2, y2, z2, a2, e2, r2) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) passing through another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) with a velocity percentage determined by the setSpeed() function, creating a circular path
- moveCircularX(x1, y1, z1, a1, e1, r1,j71, x2, y2, z2, a2, e2, r2,j72) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) and the opening position in mm of the gripper in (j71) passing throug another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) and the opening position in mm of the gripper  in (j72) with a velocity percentage determined by the setSpeed() function, creating a circular path
- moveCancel() :
Can cancel the buffer of the moves, helpful when some erros occur in generating a path trajectory which is not allowed. The cancel can be called by pressing the 'k' key, even in the blocking mode. Pressing 'k' stops the movement itself with a deceleration.
- moveToHome() :
Can move the joints to the Home position, sending a command zero to all the joints and the gripper, with a velocity percentage determined by the setSpeed() function
- moveToWaitingPos() :
Can move the joints to the Waiting position, sending a command (0,40,50,0,90,0) to the joints and 0 to the gripper, with a velocity percentage determined by the setSpeed() function
- moveGripper(j7) :
Can set the desired width of the gripper, in mm. The value of j7 should be a number between «0» and «80».
- moveGripperCart(j7) :
Can set the desired width of the gripper, in mm, by maintaining the position of the tool frame (the z axis does not change in value). The value of j7 should be a number between «0» and «80».
- moveCooperative(x, y, z, a, e, r, xf, yf, zf, af, ef, rf) :
Used in cooperative tasks. The first set of coordinates represents the robot cartesian frame. The second set of coordinates is the tool frame, common to all the arms involved in the manipulation. The parameters passed are the target position and orientation.
 - moveCooperativeX(x, y, z, a, e, r, j7, xf, yf, zf, af, ef, rf) :
Used in cooperative tasks. The first set of coordinates represents the robot cartesian frame, j7 is the gripper width [mm]. The second set of coordinates is the tool frame, common to all the arms involved in the manipulation. The parameters passed are the target position and orientation.

# JOG
- jogJoint(j1, j2, j3, j4, j5, j6, j7) :
Can move the joints, once per time for a delta expressed in degrees in the (j1, j2, j3, j4, j5, j6, j7) with a velocity percentage determined by the setSpeed() function
- jogCartesian(x, y, z, a, e, r) :
Can move the joints in a cartesian_position, once coordinate per time for a delta through the pose in (x1, y, z, a, e, r) with a velocity percentage determined by the setSpeed() function
- jogStop() :
Can stop the joint movement.

# LISTEN
- listenValues() :
Calls listenCartesianPosition and listenJointState, hence starts the subscription to both cartesian and state variables. Calls all the functions described below.
	- listenJointState() :
	Allows to start the subscribing on the topic related to these informations:
	[Position 
	Velocity
	Current]
	- listenCartesianPosition() :
	Allows to start the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)
- listenMovementAck() :
Allows to start the subscribing on the topic related to the movement aknowledge, which sends a feedback on the movement.
- waitAcknowledgment() :
Waits for the end-of-movement acknowledge, if the setting is on stepByStep (default).

# UNLISTEN
- unlistenValues() :
Calls listenCartesianPosition and listenJointState, hence starts the subscription to both cartesian and state variables. Calls all the functions described below.
	- unlistenJointState() :
	Allows to stop the subscribing on the topic related to these informations:
	[Position 
	Velocity
	Current]
	- unlistenCartesianPosition() :
	Allows to stop the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)
- unlistenMovementAck() :
Allows to stop the subscribing on the topic related to the movement aknowledge, which sends a feedback on the movement.

# GET
- getJointState() :
Allows to access the Joint State dictionary variables as lists of numbers:
                [Position 
                 Velocity
                 Current
                 cartesianPosition]
- getJoints() :
Allows to get the joint position, in degrees.
- getCartesian() :
Allows to get the cartesian position X Y Z in mm.
- getCartesianFull() :
Allows to get the cartesian position X Y Z in mm and A E R in degrees.
- getGripper() :
Allows to get the gripper opening value in mm.
- gripperWidth(toInteger) :
Returns the current width of the gripper. The toInteger parameter is optional and it is set to True by default. If set to False the function returns a float with 14 decimal values.

# FILL
This methods are the callback functions to the ROS Topic subscription. They are not intended for user purposes.
- fillJointStateValues(message) :
Receives the joints informations in terms of Position, Velocity and Current.
- fillCartesianPosition(message) :
Receives the cartesian position of the robot.
- fillMovementAck(message) :
Receives the movement acknowledgment, indicating when the movement is over.

# CONNECT
- disconnect() :
Disconnect the e.Do from the websocket, dropping the ROS connection. This is reversable: it is possible to connect again.
- connect() :
Connects the e.Do to the websocket. Should be used when we want to riconnect after the disconnect() function was called. The topics and the service connections are restablished.
- safeShutdown() :
Allows a safe shutdown for the robot. Disconnects from ROS and stops the main event loop. This is not reversable. It should be called before manually shutting down the e.Do.

# How to calibrate the robot

This example allows you to initialize, disengage the brakes and calibrate the axes, if already in the "Home" position, through a Python program.

If you have not the e.DO robot in Home Position don't use this program, connect e.DO with the e.DO App and use the standard calibration, when the robot will be calibrated disconnect the e.DO App with the proper button and connect python without init, disengage and calibrate functions, because this part has been already done.

Turn on the e.DO robot, the init_7Axes() command line should only be used once after power on, disengage_std() is useful for disengaging the brakes every time the brakes are applied, the last command line calib_axes() is very important for correct operation and must be sent when the notches of the joints are aligned (“Home position").

```python
def StartUp(myedo):
    myedo.init7Axes()
    time.sleep(10)
    myedo.disengageStd()
    time.sleep(15)
    myedo.calibAxes() # Mandatory in HOME POSITION
```


# The eduedo class

It is a class developed to facilitate the initialization and the use of the e.DO robot. It is initialized as follows:

```
myedu = eduedo(myedo)
```

and it contains the following methods.

- rodeo() :
It is a demo of e.DO moving.
- gripperOpen() :
Can open gripper with maximum width (80mm).
- gripperClose() :
Can close the gripper all the way (0mm).
- setGripperOpen(j7) :
Can open the gripper to a defined width, passed as parameter in j7. The width is expressed in millimiters, and the accepted range goes from «0» to «80».
- getMyedoState() :
Can print the joint state variables in degrees, the cartesian coordinates, and the gripper width in millimiters.
- disconnectMyedo() :
Disconnect the e.Do from the websocket, dropping the ROS connection. This is reversable: it is possible to connect again.
- connectreconnectMyedo() :
Connects the e.Do to the websocket. Should be used when we want to riconnect after the disconnect() function was called. The topics and the service connections are restablished.
- safeShutdownMyedo() :
Allows a safe shutdown for the robot. Disconnects from ROS and stops the main event loop. This is not reversable. It should be called before manually shutting down the e.Do.
- unblockMyedo() :
When the e.Do is not responding to commands, this function unblocks the robot, brings the e.Do to the waiting position, and opens the gripper.