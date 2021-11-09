import time
import math

import unittest
import roslibpy
from pyedo import eduedo

edo = eduedo('10.42.0.49')

edo.disengageStd()


class TestEduedo(unittest.TestCase):

    def setUp(self):
        if edo.getJoints() != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            edo.moveToHome() # this gets tested by first moving the joints in a different position
        if edo.getGripper() != 0.0:
            edo.moveToHome()
        self.gripperValue = 50.80  
        self.joint6axis = [10.40,90.5,63.82,25.64,99.65,88.23] # random values
        self.cartesianCoords = [405, 0, 190, 0, 180, 0] # random values

    def test_moveGripperCart(self):
        edo.moveToWaitingPos() # testing a cartesian movement
        z = edo.getCartesian()[2]
        edo.moveGripperCart(self.gripperValue)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), self.gripperValue)

        # Changing the location
        edo.moveCartesianX(400, -100, 180, 0, 180, 40)
        z = edo.getCartesian()[2]
        edo.moveGripperCart(self.gripperValue)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), self.gripperValue)

        # Trying limit values and values out of range
        edo.moveToWaitingPos()
        z = edo.getCartesian()[2]
        edo.moveGripperCart(0)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), 0)

        edo.moveCartesianX(400, -100, 180, 0, 180, 40)
        z = edo.getCartesian()[2]
        edo.moveGripperCart(80)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), 80)

        edo.moveGripperCart(-10)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), 0)

        edo.moveCartesianX(400, -100, 180, 0, 180, 40)
        z = edo.getCartesian()[2]
        edo.moveGripperCart(100)
        self.assertEqual(z, edo.getCartesian()[2])
        self.assertEqual(edo.getGripper(), 80)


    def test_moveCancel(self):
        # Cannot be in blocking modality    
        stepByStep = edo.stepByStep
        if stepByStep:
            edo.stepByStepOff()

        # The e.Do is in home position. Starting a long movement and cancelling it.
        edo.moveJoints(178,99,99,178,103,178)
        edo.moveCancel()
        while edo.movementAckValue['movementFeedback']['type'] != 2:
            pass
        time.sleep(0.5)
        joints1 = edo.getJoints()
        self.assertNotEqual(joints1, [178,99,99,178,103,178]) # checking that the movement was not completed
        time.sleep(5)
        joints2 = edo.getJoints() # taking the value after a while
        self.assertEqual(joints1, joints2) # check not to have moved

        if stepByStep:
            edo.stepByStepOn()


    def test_calibAxes(self):
        edo.moveJoints(10,10,10,10,10,10)


        edo.calibAxes()
        time.sleep(2)
        self.assertEqual([edo.getJoints()[0], edo.getJoints()[1], edo.getJoints()[2], edo.getJoints()[3], edo.getJoints()[4], edo.getJoints()[5], edo.getGripper()], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        edo.moveJoints(-10,-10,-10,-10,-10,-10)
        edo.calibAxes()
        time.sleep(2)


    def test_moveJoints(self):
        edo.moveJoints(self.joint6axis[0], self.joint6axis[1], self.joint6axis[2], self.joint6axis[3], self.joint6axis[4], self.joint6axis[5])
        time.sleep(0.5)
        self.assertEqual(edo.getJoints(), self.joint6axis)

        edo.moveJoints(0,0,0,0,0,0)
        time.sleep(0.5)
        self.assertEqual(edo.getJoints(), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        edo.moveJoints(-178,-99,-99,-178,-103,-178) # edge
        time.sleep(0.5)
        self.assertEqual(edo.getJoints(), [-178,-99,-99,-178,-103,-178])

        edo.moveJoints(178,99,99,178,103,178) # edge
        time.sleep(0.5)
        self.assertEqual(edo.getJoints(), [178,99,99,178,103,178])

        with self.assertRaises(ValueError) :
            edo.moveJoints(-190,-100,-100,-190,-110,-190)
            
        with self.assertRaises(ValueError) :
            edo.moveJoints(190,100,100,190,110,190)

        with self.assertRaises(ValueError):
            edo.moveJoints(-180, 0,0,0,0,0)

        with self.assertRaises(ValueError):
            edo.moveJoints(0, 0,100,0,0,0)


    def test_moveSingleJoint(self):
        edo.moveSingleJoint(5, self.joint6axis[4])
        time.sleep(0.5)
        self.assertEqual(edo.getJoints()[4], self.joint6axis[4])
        
        # Testing edge values
        edo.moveSingleJoint(1, self.joint6axis[0])
        time.sleep(0.5)
        self.assertEqual(edo.getJoints()[0], self.joint6axis[0])
        
        edo.moveSingleJoint(6, self.joint6axis[5])
        time.sleep(0.5)
        self.assertEqual(edo.getJoints()[5], self.joint6axis[5])

        with self.assertRaises(ValueError):
            edo.moveSingleJoint(10,0)

        with self.assertRaises(ValueError):
            edo.moveSingleJoint(1,190)

        with self.assertRaises(ValueError):
            edo.moveSingleJoint(2, -100)


    def test_moveCartesian(self):
        edo.moveJoints(0,40,50,0,90,0)
        edo.moveCartesian(self.cartesianCoords[0], self.cartesianCoords[1],self.cartesianCoords[2],self.cartesianCoords[3],self.cartesianCoords[4],self.cartesianCoords[5])
        self.assertEqual(edo.getCartesianFull(), self.cartesianCoords)

        edo.moveToHome()
        time.sleep(0.5)
        with self.assertRaises(ValueError):
            edo.moveCartesian(600, 500, 800, 0, 0, 0)


    def test_moveCartesianX(self):
        edo.moveJoints(0,40,50,0,90,0)
        edo.moveCartesianX(self.cartesianCoords[0], self.cartesianCoords[1],self.cartesianCoords[2],self.cartesianCoords[3],self.cartesianCoords[4],self.cartesianCoords[5], self.gripperValue)
        self.assertEqual([edo.getCartesianFull(), edo.getGripper()], [self.cartesianCoords, self.gripperValue])

        edo.moveJoints(0,40,50,0,90,0)
        edo.moveCartesianX(self.cartesianCoords[0], self.cartesianCoords[1],self.cartesianCoords[2],self.cartesianCoords[3],self.cartesianCoords[4],self.cartesianCoords[5], 90)
        self.assertEqual([edo.getCartesianFull(), edo.getGripper()], [self.cartesianCoords, 80])

        edo.moveJoints(0,40,50,0,90,0)
        edo.moveCartesianX(self.cartesianCoords[0], self.cartesianCoords[1],self.cartesianCoords[2],self.cartesianCoords[3],self.cartesianCoords[4],self.cartesianCoords[5], -90)
        time.sleep(0.2)
        self.assertEqual([edo.getCartesianFull(), edo.getGripper()], [self.cartesianCoords, 0])

        edo.moveToHome()
        time.sleep(0.5)
        with self.assertRaises(ValueError):
            edo.moveCartesianX(600, 500, 800, 0, 0, 0, 5)
        

    
    def test_moveToHome(self):
        # first we move it away from home
        edo.moveJoints(self.joint6axis[0], self.joint6axis[1], self.joint6axis[2], self.joint6axis[3], self.joint6axis[4], self.joint6axis[5])
        time.sleep(0.5)
        edo.moveToHome()
        time.sleep(0.5)
        self.assertEqual([0.0,0.0,0.0,0.0,0.0,0.0], [round(float(i),2) for i in edo.jointStateValues.get('jointPosition')[0:6]])
    
    def test_moveToWaitingPos(self):
        edo.moveToWaitingPos()
        time.sleep(0.5)
        self.assertEqual([0.0,40.0,50.0,0.0,90.0,0.0], [round(float(i),2) for i in edo.jointStateValues.get('jointPosition')[0:6]])
    

    def test_moveGripper(self):
        edo.moveGripper(self.gripperValue)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), self.gripperValue)

        edo.moveGripper(0)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), 0)

        edo.moveGripper(80)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), 80)

        edo.moveGripper(-10)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), 0)

        edo.moveGripper(90)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), 80)


    def test_gripperWidth(self):
        edo.moveGripper(50.8)
        time.sleep(0.5)
        self.assertEqual(math.floor(edo.getGripper()), edo.gripperWidth())

        edo.moveGripper(20.74)
        time.sleep(0.5)
        self.assertEqual(edo.getGripper(), edo.gripperWidth(False))

        edo.moveGripper(90)
        time.sleep(0.5)
        self.assertEqual(80.0, edo.gripperWidth(False))

        edo.moveGripper(-20)
        time.sleep(0.5)
        self.assertEqual(0.0, edo.gripperWidth(False))


    def test_jogJoint(self):
        temp = edo.getJoints()
        delta = 5.0
        for i in range(0,20):
            edo.jogJoint(delta, delta, delta, delta, delta, delta, delta)
            time.sleep(0.5)
        
        self.assertNotEqual(edo.getJoints(), temp)
    
    
    def test_jogCartesian(self):
        edo.moveJoints(0,40,50,0,90,0)
        temp = edo.getCartesianFull()
        delta = 5.0
        for i in range(0,20):
            edo.jogCartesian(delta,0,0,0,0,0)
            time.sleep(0.5)
        
        self.assertNotEqual(edo.getCartesianFull(), temp)

        edo.moveToHome()

        with self.assertRaises(ValueError):
            edo.jogCartesian(delta, delta, delta, delta, delta, delta)


    def test_setSpeed(self):
        edo.setSpeed(45.8)
        self.assertEqual(edo.commandTemplate['ovr'], 45.8)

        edo.setSpeed(100)
        self.assertEqual(edo.commandTemplate['ovr'], 100)

        edo.setSpeed(120)
        self.assertEqual(edo.commandTemplate['ovr'], 100)

        edo.setSpeed(0)
        self.assertEqual(edo.commandTemplate['ovr'], 0)

        edo.setSpeed(-10)
        self.assertEqual(edo.commandTemplate['ovr'], 0)


    def test_getJoints(self):
        self.assertEqual(edo.getJoints(), [round(float(i),2) for i in edo.jointStateValues.get('jointPosition')][0:6])

    
    def test_getCartesian(self):
        self.assertEqual(edo.getCartesian(), [round(float(i),2) for i in edo.jointStateValues.get('cartesianPosition')][0:3])


    def test_getCartesianFull(self):
        self.assertEqual(edo.getCartesianFull(), [round(float(i),2) for i in edo.jointStateValues.get('cartesianPosition')])


    def test_getGripper(self):
        self.assertEqual(edo.getGripper(), edo.jointStateValues.get('jointPosition')[6])


    def test_getJointState(self):
        self.assertEqual([round(float(i),2) for i in edo.jointStateValues['cartesianPosition']], edo.getJointState()[0])
        self.assertEqual([round(float(i),2) for i in edo.jointStateValues['jointPosition']], edo.getJointState()[1])
        self.assertEqual([round(float(i),2) for i in edo.jointStateValues['velocity']], edo.getJointState()[2])
        self.assertEqual([round(float(i),2) for i in edo.jointStateValues['current']], edo.getJointState()[3])


    def test_listenJointState(self):
        edo.JointState.unsubscribe() # First we need to unsubscribe from the topic
        time.sleep(0.5)

        edo.listenJointState()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/usb_jnt_state', nodeList)


    def test_listenCartesianPosition(self):
        edo.CartesianPosition.unsubscribe() # First we need to unsubscribe from the topic
        time.sleep(0.5)

        edo.listenCartesianPosition()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/cartesian_pose', nodeList)


    def test_listenMovementAck(self):
        edo.MovementFeedback.unsubscribe() # First we need to unsubscribe from the topic
        time.sleep(0.5)

        edo.listenMovementAck()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/machine_movement_ack', nodeList)


    def test_listenValues(self):
        edo.MovementFeedback.unsubscribe() # First we need to unsubscribe from the topics
        edo.JointState.unsubscribe()
        edo.CartesianPosition.unsubscribe()
        time.sleep(0.5)

        edo.listenValues()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/machine_movement_ack', nodeList)

        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/cartesian_pose', nodeList)

        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertIn('/usb_jnt_state', nodeList)


    def test_unlistenJointState(self):
        edo.JointState.subscribe(edo.fillJointStateValues) # First we need to subscribe to the topic
        time.sleep(0.5)

        edo.unlistenJointState()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/usb_jnt_state', nodeList)
    
        edo.listenJointState() # Otherwise the other tests might fail


    def test_unlistenCartesianPosition(self):
        edo.CartesianPosition.subscribe(edo.fillCartesianPosition) # First we need to subscribe to the topic
        time.sleep(0.5)

        edo.unlistenCartesianPosition()
        time.sleep(1)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/cartesian_pose', nodeList)

        edo.listenCartesianPosition() # Otherwise the other tests might fail


    def test_unlistenMovementAck(self):
        edo.MovementFeedback.subscribe(edo.fillMovementAck) # First we need to subscribe to the topic
        time.sleep(0.5)

        edo.unlistenMovementAck()
        time.sleep(1)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/machine_movement_ack', nodeList)

        edo.listenMovementAck() # Otherwise the other tests might fail


    def test_unlistenValues(self):
        edo.MovementFeedback.subscribe(edo.fillMovementAck) # First we need to subscribe to the topic
        edo.JointState.subscribe(edo.fillJointStateValues)
        edo.CartesianPosition.subscribe(edo.fillCartesianPosition)
        time.sleep(0.5)

        edo.unlistenValues()
        time.sleep(0.5)
        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/machine_movement_ack', nodeList)

        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/cartesian_pose', nodeList)

        nodeList = edo.client.get_node_details('/rosbridge_websocket')['subscribing']
        self.assertNotIn('/usb_jnt_state', nodeList)
        
        edo.listenValues() # Otherwise the other tests might fail
        time.sleep(0.5)

    def test_disconnect(self):
        connected = edo.client.is_connected
        if not connected:
            edo.client.connect()
            time.sleep(2)
        
        edo.disconnect()
        self.assertFalse(edo.client.is_connected)
        
        if connected:
            edo.client.connect()
            time.sleep(2)

    def test_connect(self):
        connected = edo.client.is_connected
        if connected:
            edo.client.close()
            time.sleep(2)
        
        edo.connect()
        self.assertTrue(edo.client.is_connected)
        
        if not connected:
            edo.client.close()
            time.sleep(2)


if __name__ == '__main__':
    unittest.main()
