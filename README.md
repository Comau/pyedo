# pyedo
This package contains the SDK to program e.DO robot with Python

# Programming e.DO with Python
Install the latest Python version on your device:

Download the SDK from our site. Drag and drop the "pyedo" package into the python path site packages.

"C:\Users\YourAccount\AppData\Local\Programs\Python\PythonXX\Lib\site-packages\"

You are now able to connect your device with e.DO via Python.
Create a new program, import the object from the pyedo library, create an instance of an eDO object in your program and now move e.DO with the available methods defined below.

# INIT
This method allows to initialize the e.DO robot, with or without gripper:
- init_7Axes() :
can initialize the robot with 7 axes (e.DO 6 axes plus the gripper)
- init_6Axes() :
can initialize the robot with 6 axes 

# DISENGAGE
This method allows to disengage the brakes:
- disengage_std() :
can disengage the robot with the standard movement (RECOMMENDED)
- disengage_safe() :
can release only the brakes
- disengage_sin() :
can disegage the robot with a sinusoidal movement (MANDATORY TO HAVE THE JOINTS FAR AWAY FROM THE STROKE END)

# CALIB
This method allows to calibrate all the joints:
- calib_axes() :
Can calibrate all the robot axes (MANDATORY TO HAVE THE JOINTS ALLIGNED WITH THE NOTCHES (Home Position)) 
The calibration is possible only after the initizialization(init method) and the disengage of the brakes(disengage method) .

# MOVE
This method allows to move the e.DO robot, performing the pianification of the trajectory in different ways:
- move_joint(ovr, j1, j2, j3, j4, j5, j6, j7) :
Can move the joints to a joint_position indicated through the angles in degrees (j1, , j2, j3, j4, j5, j6) and the opening of the gripper J7 in mm with a velocity percentage in (ovr) up to the maximum value «100»
- move_cartesian(ovr, x, y, z, a, e, r) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) with a velocity percentage in (ovr) up to the maximum value «100»
- move_circular(ovr, x1, y1, z1, a1, e1, r1, x2, y2, z2, a2, e2, r2) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) passing through another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) with a velocity percentage in (ovr), up to the maximum value «100», creating a circular path.
- move_cancel() :
Can cancel the buffer of the moves, helpfull when some erros occur in generating path trajectory not allowed.
- move_cartesianX(ovr, x, y, z, a, e, r, j7) :
Can move the joints to a cartesian_position indicated through the pose in (x, y, z, a, e, r) and the opening position in mm of the gripper  in (j7) with a velocity percentage in (ovr) up to the maximum value "100"
- move_circularX(ovr, x1, y1, z1, a1, e1, r1,j71, x2, y2, z2, a2, e2, r2,j72) :
Can move the joints to a cartesian_position indicated through the pose in (x1, y1, z1, a1, e1, r1) and the opening position in mm of the gripper  in (j71) passing throug another cartesian_position indicated through the pose in (x2, y2, z2, a2, e2, r2) and the opening position in mm of the gripper  in (j72)  with a velocity percentage in (ovr), up to the maximum value "100", creating a circular path.

# JOG
- jog_joint(ovr, j1, j2, j3, j4, j5, j6, j7) :
Can move the joints, once per time for a delta, in a direction indicated with  "1"(positive) or "-1"(negative) in the (j1, j2, j3, j4, j5, j6, j7) with a velocity percentage in (ovr) up to the maximum value "100"
- jog_cartesian(ovr, x, y, z, a, e, r) :
Can move the joints in a cartesian_position, once coordinate per time for a delta, indicated with "1"(positive) or "-1"(negative) in the a cartesian_position through the pose in (x1, y, z, a, e, r) with a velocity percentage in (ovr) up to the maximum value "100"

# LISTEN
- listen_JointState() :
Allows to start the subscribing on the topic related to these informations:
[Position 
Velocity
Current]
- listen_CartesianPosition() :
Allows to start the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)

# UNLISTEN
- unlisten_JointState() :
Allows to stop the subscribing on the topic related to these informations:
[Position 
Velocity
Current]
- unlisten_CartesianPosition() :
Allows to stop the subscribing on the topic related to the cartesian position (x,y,z,a,e,r)

# GET
- get_JointState() :
Allows to get the Joint State dictionary containing the variables listened with the listen method:
[Position 
Velocity
Current
cartesianPosition]

