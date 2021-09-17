Pyedo
=======================================

This package contains the SDK to program e.DO robot with Python
<img src="https://edo.cloud/wp-content/uploads/2018/12/edo-home.png" alt="pyedo logo" width="100px"/> 


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
can initialize the robot with 7 axes (e.DO 6 axes plus the gripper)
- init6Axes() :
can initialize the robot with 6 axes 

# DISENGAGE
This method allows to disengage the brakes:
- disengageStd() :
can disengage the robot with the standard movement (RECOMMENDED)
- disengageSafe() :
can release only the brakes
- disengageSin() :
can disegage the robot with a sinusoidal movement (MANDATORY TO HAVE THE JOINTS FAR AWAY FROM THE STROKE END)

# CALIB
This method allows to calibrate all the joints:
- calibAxes() :
Can calibrate all the robot axes (MANDATORY TO HAVE THE JOINTS ALLIGNED WITH THE NOTCHES (Home Position)) 
The calibration is possible only after the initizialization(init method) and the disengage of the brakes(disengage method) .

# MOVE
This method allows to move the e.DO robot, performing the pianification of the trajectory in different ways:
- moveJoint(ovr, j1, j2, j3, j4, j5, j6, j7) :
Can move the joints to a joint_position indicated through the angles in degrees (j1, , j2, j3, j4, j5, j6) and the opening of the gripper J7 in mm with a velocity percentage in (ovr) up to the maximum value «100»
- moveCartesian(ovr, x, y, z, a, e, r) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) with a velocity percentage in (ovr) up to the maximum value «100»
- moveCircular(ovr, x1, y1, z1, a1, e1, r1, x2, y2, z2, a2, e2, r2) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) passing through another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) with a velocity percentage in (ovr), up to the maximum value «100», creating a circular path.
- moveCancel() :
Can cancel the buffer of the moves, helpfull when some erros occur in generating path trajectory not allowed.
- moveCartesianX(ovr, x, y, z, a, e, r, j7) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) and the opening position in mm of the gripper  in (j7) with a velocity percentage in (ovr) up to the maximum value "100"
- moveCircularX(ovr, x1, y1, z1, a1, e1, r1,j71, x2, y2, z2, a2, e2, r2,j72) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) and the opening position in mm of the gripper  in (j71) passing throug another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) and the opening position in mm of the gripper  in (j72)  with a velocity percentage in (ovr), up to the maximum value "100", creating a circular path.

# JOG
- jogJoint(ovr, j1, j2, j3, j4, j5, j6, j7) :
Can move the joints, once per time for a delta, in a direction indicated with  "1"(positive) or "-1"(negative) in the (j1, j2, j3, j4, j5, j6, j7) with a velocity percentage in (ovr) up to the maximum value "100"
- jogCartesian(ovr, x, y, z, a, e, r) :
Can move the joints in a cartesian_position, once coordinate per time for a delta, indicated with "1"(positive) or "-1"(negative) in the a cartesian_position through the pose in (x1, y, z, a, e, r) with a velocity percentage in (ovr) up to the maximum value "100"

# LISTEN
- listenJointState() :
Allows to start the subscribing on the topic related to these informations:
[Position 
Velocity
Current]
- listen_CartesianPosition() :
Allows to start the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)

# UNLISTEN
- unlistenJointState() :
Allows to stop the subscribing on the topic related to these informations:
[Position 
Velocity
Current]
- unlisten_CartesianPosition() :
Allows to stop the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)

# GET
- getJointState() :
Allows to get the Joint State dictionary containing the variables listened with the listen method:
[Position 
Velocity
Current
cartesianPosition]

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
