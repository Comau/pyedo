PYEDO TEST
=======================================

This module tests the [pyedo package](https://github.com/Comau/pyedo)

# USAGE

About the test
-----------
The library used for building the tests is [unittest](https://docs.python.org/3/library/unittest.html), already present in the standard Python installation.
For any doubts on the testing process: Automated Testing in Python.


How to use the test module
-----------
Download the script and **launch the rosapi node**, from the ROS terminal:

        $ rosrun rosapi rosapi_node
		
Run the test_edo script. The process will go for a couple of minutes.

The output you will obtain, if all the functions are running correctly, is something like the following :

```
Connected with e.DO
e.DO with Gripper has been initialized
e.DO is ready to have fun!
.Connected to ROS.
.Disconnected from ROS: can connect to the app.
......ATTENTION. Max gripper width = 80 mm.
Gripper will open to 80 mm.
ATTENTION. Min gripper width = 0 mm.
Gripper will close to 0 mm.
.........Attention: the gripper will open to 80mm
Attention: the gripper will close to 0
.ATTENTION. Min gripper width = 0 mm.
Gripper will close to 0 mm.
ATTENTION. Max gripper width = 80 mm.
Gripper will open to 80 mm.
.Attention: the gripper will close to 0
Attention: the gripper will open to 80mm
.....ATTENTION. Max speed = 100%.
ATTENTION. Min speed = 0%.
.....
----------------------------------------------------------------------
Ran 29 tests in 235.132s

OK
```

*Remember: tests are not always executed in the same order*

If the output gives failures for the *unlisten* set of functions it can mean that the subscribe request is coming externally. Check that the application connected to e.Do is not open.
If the output gives other kind of failures it means that there is an error in some pyedo function.
If, on the other hand, it gives an error **E** statement there is something wrong with the test. In this case first check the connection to ROS, and that the rosapi node is running correctly.

# TESTS:

The setUp() method will run before each test, preparing the environment.

## Calibration Functions
- test_calibAxes() :
The test makes the joints and the gripper perform a movement of value 10 (deg or mm), then the `edo.calibAxes()` is called and the new position of the axes is compared with the (0,0,0,0,0,0,0) vector, to check that the calibration worked properly. The joints are then recalibrated in -10, in order to bring the situation back to normal and be able to use the e.Do. This second step is necessary for when we are working with a real robot.

## Move Functions
- test_moveJoints() :
The movement to a random pose is tested, as well as the movement to the coordinates (0,0,0,0,0,0) and the joints limit values (178,99,99,178,103,178) and (-178,-99,-99,-178,-103,-178). The test also assures that when a not acceptable value is given as input to one or all the joints an error is correctly raised.
- test_moveSingleJoint() :
The movement of a joint to a specific value is tested. It is also assured that errors are raised when passing a non existing joint number (eg. 10) or a value out of range for the selected joint.
- test_moveToHome() :
Joints are moved away from (0,0,0,0,0,0) and then the `edo.moveToHome()` function is tested.
- test_moveToWaitingPos() :
The `edo.moveToWaitingPos()` function is tested by comparing the joint values with the position (0,40,50,0,90,0).
- test_moveGripper() :
The gripper is opened to a specified value, then to its minimum value 0, and to its maximum value 80. Some values out of range are tested, to make sure that the function correctly sets the command to the maximum or minimum accepted values.
- test_moveCartesian() :
A `edo.moveJoints()` is called to bring the robot in a suitable configuration (exit singularity). The function is then tested. The robot is brought to a singularity configuration (the home position) and it is tested whether the error is raised correctly.
- test_moveCartesianX() :
A `edo.moveJoints()` is called to bring the robot in a suitable configuration (exit singularity). The function is then tested. Some values out of range for the gripper are tested. The robot is brought to a singularity configuration (the home position) and it is tested whether the error is raised correctly.
- test_moveCancel() :
The stepByStep modality is deactivated as to be able to give other inputs, then a long movement is given as input to the joints (from home to joint limits). The `edo.moveCancel()` is immediately called and the values taken from the joints are compared in different moments to be sure that the robot has not reached the target and is not moving.
- test_moveGripperCart() :
It is tested that when opening and closing the gripper, the cartesian value on z stays the same. The test is executed in the waiting position, in a random position, for movements at the limit of 80mm and 0 (fully closed), and for values out of range.

## Set Functions
- test_setSpeed() :
The function `edo.setSpeed(ovr)` is passed a random float value. The value is then compared with the variable accessed directly through `edo.commandTemplate['ovr']`. Then the values 0 and 100 are tested, as well as some values which are not in the acceptable range.

## Get Functions
- test_getJointState() :
The function is tested by comparing its output element by element with the value accessed directly through the `edo.jointStateValues` variable.
- test_getJoints() :
The function is tested by comparing its output with the value accessed directly through `edo.jointStateValues.get('jointPosition')`.
- test_getCartesian() :
The function is tested by comparing its output with the value accessed directly through `edo.jointStateValues.get('cartesianPosition')][0:3]`.
- test_getCartesianFull() :
The function is tested by comparing its output with the value accessed directly through `edo.jointStateValues.get('cartesianPosition')]`.
- test_getGripper() :
The function is tested by comparing its output with the value accessed directly through `edo.jointStateValues.get('jointPosition')[6]`.
- test_gripperWidth() :
The function is tested with a random float value of the gripper. The `edo.gripperWidth()` function is tested in its int setting and in its float setting. Movements of the gripper out of the range of accepted values are tested, to make sure that the information is correctly read by the tested function.

## Jog Functions
- test_jogJoint() :
The function is called inside a loop. When the loop ends the values are checked: they should not be equal to the initial values.
- test_jogCartesian() :
The joints are moved to the waiting position and the function is tested. The joints are then moved to the home position and it is checked that the error is raised correctly.

## Listen Functions
- test_listenJointState() :
First the unsubscribe from the topic /usb_jnt_state is called, then the function `edo.listenJointState()` is called. Through the roslibpy API function `get_node_details()` the list of topics to which the e.Do node is subscribed is obtained, and the presence of /usb_jnt_state is checked.
- test_listenCartesianPosition() :
As for `test_listenJointState()`, the unsubscribe is called on the topic /cartesian_pose. Then the function `edo.listenCartesianPosition()` is called and the presence of the topic /cartesian_pose in the e.Do node subscribed list is checked.
- test_listenMovementAck() :
As before, the unsubscribe is called on the topic /machine_movement_ack. Then the function `edo.listenMovementAck()` is called and the presence of the topic /machine_movement_ack in the e.Do node subscribed list is checked.
- test_listenValues() :
The unsubscribe is called on the topics /machine_movement_ack , /usb_jnt_state and /cartesian_pose. Then, the function `edo.listenValues()` is called and we check the presence of the topics in the node list.

## Unlisten Functions
- test_unlistenJointState() :
First the subscribe to the topic /usb_jnt_state is called, then the function `edo.unlistenJointState()` is called. Through the roslibpy API function `get_node_details()` the list of topics to which the e.Do node is subscribed is obtained, and the absence of /usb_jnt_state is checked.
- test_unlistenCartesianPosition() :
As for `test_unlistenJointState()`, the subscribe is called on the topic /cartesian_pose. Then the function `edo.unlistenCartesianPosition()` is called and the absence of the topic /cartesian_pose in the e.Do node subscribed list is checked.
- test_unlistenMovementAck() :
The subscribe is called on the topic /machine_movement_ack. Then the function `edo.unlistenMovementAck()` is called and the absence of the topic /machine_movement_ack in the e.Do node subscribed list is checked.
- test_unlistenValues() :
The subscribe is called on the topics /machine_movement_ack , /usb_jnt_state and /cartesian_pose. Then, the function `edo.unlistenValues()` is called and we check the absence of the topics in the node list.

## Disconnect Functions
- test_disconnect() :
First registers whether the node is connected to ROS: if not connects with `edo.client.connect()`. The function is called and then the connection is checked as False. Eventually, the node is reconnected to ROS if it was connected originally.
- test_connect() :
First registers whether the node is connected to ROS: if so the connection is closed with `edo.client.close()`. The function is called and then the connection is checked as True. Eventually, the node is disconnected to ROS if it was not connected originally.
