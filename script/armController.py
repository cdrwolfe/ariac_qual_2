#!/usr/bin/env python
import sys
import numpy
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import dynamic_reconfigure.client


# Arm controller module, handles functions associated with arm control, tasks etc
class ArmController:
    # An arm controller class
    def __init__(self, operators):
        # Initialise
        self.op = operators
        self.pastArmState = None
        # We need a tf listener to convert poses into arm reference base
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Initialise moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # Create move group for manipulator
        self.groupManipulator = moveit_commander.MoveGroupCommander("manipulator")
        # Allow replanning to increase the odds of a solution
        self.groupManipulator.allow_replanning(True)
        # Allow 5 seconds per planning attempt
        self.groupManipulator.set_planning_time(5)
        # Set goal joint tolerance
        self.groupManipulator.set_goal_joint_tolerance(0.005)
        # Set goal goal tolerance
        self.groupManipulator.set_goal_tolerance(0.005)
        # Set goal goal tolerance
        self.groupManipulator.set_goal_position_tolerance(0.005)
        # Set goal orientation tolerance
        self.groupManipulator.set_goal_orientation_tolerance(0.005)
        # Define end effector group
        self.groupEffector = moveit_commander.MoveGroupCommander("endeffector")
        # Set trajectory execution ros parameters to disabled
        client = dynamic_reconfigure.client.Client('move_group/trajectory_execution/')
        params = { 'allowed_start_tolerance' : '0.0'}
        config = client.update_configuration(params)

    def armPose(self):
        # Return current arm pose
        pose = self.groupManipulator.get_current_pose()
        # Return
        return pose

    def poseTarget(self, target):
        # Given a target try and position arm end effector over it, return bool
        poseTarget = geometry_msgs.msg.Pose()
        poseTarget.position.x = target.pose.position.x
        poseTarget.position.y = target.pose.position.y
        poseTarget.position.z = target.pose.position.z
        poseTarget.orientation = target.pose.orientation
        self.groupManipulator.set_pose_target(poseTarget)

    def positionTarget(self, target):
        # Given a target try and position arm end effector over it
        xyz = [0,0,0]
        xyz[0] = target.pose.position.x 
        xyz[1] = target.pose.position.y
        xyz[2] = target.pose.position.z
        # Send
        self.groupManipulator.set_position_target(xyz)

    def planPose(self):
        # Plan arm pose 
        plan = self.groupManipulator.plan()

    def executePlan(self):
        # Execute desired plan
        self.groupManipulator.go(wait=True)

    def transformPose(self, pose, posOffset, orienOffset, frame):
        # Transform pose using frame
        targetPose = PoseStamped()
        targetPose.header.frame_id = frame
        targetPose.pose.position.x = pose.position.x
        targetPose.pose.position.y = pose.position.y
        targetPose.pose.position.z = pose.position.z
        targetPose.pose.orientation.x = 0
        targetPose.pose.orientation.y = 0
        targetPose.pose.orientation.z = 0
        targetPose.pose.orientation.w = 0      
        #rospy.loginfo("Target: ")
        #rospy.loginfo(targetPose)
        # Set tranform from frame to world
        transform = self.tf_buffer.lookup_transform('world',
        frame, # source frame
        rospy.Time(0), # get the tf at first available time
        rospy.Duration(2.0)) # wait for 1 second
        # Transform
        transformedPose = do_transform_pose(targetPose, transform)
        # Offset position and orientation
        transformedPose.pose.position.z += posOffset[2]
        transformedPose.pose.orientation.x = 0.5 
        transformedPose.pose.orientation.y = 0.5 
        transformedPose.pose.orientation.z = -0.5 
        transformedPose.pose.orientation.w = 0.5 
        self.op.poseMarker(transformedPose)
        # Return
        return transformedPose
    
    def partPose(self, frame, posOffset):
        # Return the tray pose
        targetPose = PoseStamped()
        self.tf_listener.waitForTransform('world', frame, targetPose.header.stamp, rospy.Duration(4.0))
        (trans, rot) = self.tf_listener.lookupTransform('world', frame, rospy.Time(0))
        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = trans[0] + posOffset[0]
        pose.pose.position.y = trans[1] + posOffset[1]
        pose.pose.position.z = trans[2] + posOffset[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        pose = self.tf_listener.transformPose('world', pose)
        # Return
        return pose

    def home(self):
        # Set arm position to 'home' position set in SRDF file
        rospy.loginfo("Set Arm: home")
        # Set
        self.groupManipulator.set_named_target('home')
        # Execute
        self.executePlan()

    def up(self):
        # Set arm position to 'up' position set in SRDF file
        rospy.loginfo("Set Arm: up")
        # Set
        self.groupManipulator.set_named_target('up')
        # Execute
        self.executePlan()

    def handlePart(self):
        # Set joint positions to a pose for handling part
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [0, 3.14, -2.39, 2.39, 3.14, -1.51, 0.0]
        # Create a joint trajectory message and fill with values
        # Create names
        names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Create points
        point = JointTrajectoryPoint()
        point.positions = [2.39, 0.0, -2.39, 3.14, 3.14, -1.51, 0.0]
        point.time_from_start = rospy.Duration(3.0)
        # Execute
        self.op.jointTrajectory('Handle Part', names, point)

    def trayPose(self, desiredPose, frame, inventory):
        # Return the tray pose
        if frame == 'logical_camera_5_kit_tray_1_frame':
            # Return pose of agv on left
            transform = self.tf_buffer.lookup_transform('world',
            frame, # target frame, we keep this xyz orientation / direction when (0,0,0)
            rospy.Time(0), # get the tf at first available time
            rospy.Duration(2.0)) # wait for 1 second
            # Set tray camera
            trayCamera = 'logical_camera_5'
        else:
            # Return pose of agv on right
            frame = 'logical_camera_6_kit_tray_1_frame'
            transform = self.tf_buffer.lookup_transform('world',
            frame, # target frame
            rospy.Time(0), # get the tf at first available time
            rospy.Duration(2.0)) # wait for 1 second
            # Set tray camera
            trayCamera = 'logical_camera_6'
        # When transforming orientation, you must set the base pose as that of the world frame (0,0,0,1)
        # Calculate position to place on tray
        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position = desiredPose.position
        pose.pose.orientation = desiredPose.orientation
        #pose.pose.orientation.z += numpy.pi/2
        # Transform
        positionPose = do_transform_pose(pose, transform)
        self.op.poseMarker(positionPose)
        print positionPose
        #raw_input("Press Enter to continue...")
        # Get part currently being placed
        tray = inventory.buildTrayInventory(trayCamera)
        part = inventory.getTrayPart(tray, trayCamera)
        print part
        #raw_input("Press Enter to continue...")
        # Pass values into pose stamped
        armPose = PoseStamped()
        armPose.header.frame_id = frame
        armPose.pose.position.x = part['Pose'].position.x
        armPose.pose.position.y = part['Pose'].position.y
        armPose.pose.position.z = part['Pose'].position.z
        armPose.pose.orientation.x = part['Pose'].orientation.x
        armPose.pose.orientation.y = part['Pose'].orientation.y
        armPose.pose.orientation.z = part['Pose'].orientation.z
        armPose.pose.orientation.w = part['Pose'].orientation.w      
        # Create transform, this takes the current pose from camera and essentailly sets it to world pose instaead of relative to camera
        transform = self.tf_buffer.lookup_transform('world',
        trayCamera + '_frame', # target frame, we keep this xyz orientation / direction when (0,0,0)
        rospy.Time(0), # get the tf at first available time
        rospy.Duration(2.0)) # wait for 2 seconds
        # Transform
        partPose = do_transform_pose(armPose, transform)
        self.op.poseMarker(partPose)
        print partPose
        #raw_input("Press Enter to continue...")
        # Calculate differance
        rotation = self.getPoseDifferance(partPose.pose, positionPose.pose)
        # Now create target position for ee_link
        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.orientation.w = 1 # This sets our pose to match current world pose
        # Create transform, this takes the current pose from camera and essentailly sets it to world pose instaead of relative to camera
        transform = self.tf_buffer.lookup_transform('world',
        'ee_link', # target frame, we keep this xyz orientation / direction when (0,0,0)
        rospy.Time(0), # get the tf at first available time
        rospy.Duration(2.0)) # wait for 2 seconds
        # Transform
        placePose = do_transform_pose(pose, transform)
        self.op.poseMarker(placePose)
        #raw_input("Press Enter to continue...")
        # Now rotate our place pose by desired amount
        placeEuler = self.eulerFromQuaternion(placePose.pose)
        placeEuler[0] -= rotation[0]
        placeEuler[1] -= rotation[1]
        placeEuler[2] -= rotation[2]
        print placeEuler
        # Convert back to quaternion
        placeQuat = self.quaternionFromEuler(placeEuler[0], placeEuler[1], placeEuler[2])
        # Set new orientation
        placePose.pose.orientation.x = placeQuat[0]
        placePose.pose.orientation.y = placeQuat[1]
        placePose.pose.orientation.z = placeQuat[2]
        placePose.pose.orientation.w = placeQuat[3]
        self.op.poseMarker(placePose)
        #raw_input("Press Enter to continue...")
        # Now set position
        placePose.pose.position = positionPose.pose.position
        self.op.poseMarker(placePose)
        print placePose.pose.position
        #raw_input("Press Enter to continue...")
        # Add part to tray list to indicate that we have placed this part
        # Pass only values as we dont want reference
        part['Pose'].position.x = placePose.pose.position.x
        part['Pose'].position.y = placePose.pose.position.y
        part['Pose'].position.z = placePose.pose.position.z
        part['Pose'].orientation.x = placePose.pose.orientation.x
        part['Pose'].orientation.y = placePose.pose.orientation.y
        part['Pose'].orientation.z = placePose.pose.orientation.z
        part['Pose'].orientation.w = placePose.pose.orientation.w
        # Check list
        if trayCamera not in inventory.trayPartList:
            inventory.trayPartList[trayCamera] = []
            # Add
            inventory.trayPartList[trayCamera].append(part)
        else:
            # Add
            inventory.trayPartList[trayCamera].append(part)
        #raw_input("Press Enter to continue...")
        # Return
        return placePose           

    def getFallenTrayPart(self, frame, inventory):
        # Scan for any part not in tray list and pick up
        # Return the tray pose
        if frame == 'logical_camera_5_kit_tray_1_frame':
            # Set tray camera
            trayCamera = 'logical_camera_5'
        else:
            # Return pose of agv on right
            frame = 'logical_camera_6_kit_tray_1_frame'
            # Set tray camera
            trayCamera = 'logical_camera_6'
        # Get part currently being placed
        tray = inventory.buildTrayInventory(trayCamera)
        part = inventory.getFallenTrayPart(tray, trayCamera)
        # Pass values into pose stamped
        trayPose = PoseStamped()
        trayPose.header.frame_id = frame
        trayPose.pose.position.x = part['Pose'].position.x
        trayPose.pose.position.y = part['Pose'].position.y
        trayPose.pose.position.z = part['Pose'].position.z
        trayPose.pose.orientation.x = part['Pose'].orientation.x
        trayPose.pose.orientation.y = part['Pose'].orientation.y
        trayPose.pose.orientation.z = part['Pose'].orientation.z
        trayPose.pose.orientation.w = part['Pose'].orientation.w
        # Create transform, this takes the current pose from camera and essentailly sets it to world pose instaead of relative to camera
        transform = self.tf_buffer.lookup_transform('world',
        trayCamera + '_frame', # target frame, we keep this xyz orientation / direction when (0,0,0)
        rospy.Time(0), # get the tf at first available time
        rospy.Duration(2.0)) # wait for 2 seconds
        # Transform
        partPose = do_transform_pose(trayPose, transform)
        partPose.pose.orientation.x = 0.5
        partPose.pose.orientation.y = 0.5
        partPose.pose.orientation.z = -0.5
        partPose.pose.orientation.w = 0.5
        # Return
        return partPose
        

    def moveToTray(self, direction):
        # Move to either left or right tray
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [2.1, 1.54, -2.39, 2.39, 3.14, -1.51, 0.0]
        # and for Rght
        # [-2.1, -1.54, -2.39, 2.39, 3.14, -1.51, 0.0]
        if direction == 'Left':
            # Move left
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [2.39, 2.1, -2.39, 1.54, 3.14, -1.51, 0.0]
            point.time_from_start = rospy.Duration(3.0)
            # Execute
            self.op.jointTrajectory('Move to left tray', names, point)
        else:
            # Move right
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [2.39, -2.1, -2.39, 4.62, 3.14, -1.51, 0.0]
            point.time_from_start = rospy.Duration(3.0)
            # Execute
            self.op.jointTrajectory('Move to right tray', names, point)

    def moveOverTray(self, direction):
        # Position arm over tray and in place to simply drop part
        # Linear_Joint, Shoulder_Pan, Shoulder_Lift, Elbow, Wrist_1, Wrist_2, Wrist_3
        # [2.10, 1.41, -0.51, 1.26, 3.90, -1.51, 0.0]
        if direction == 'Left':
            # Move left
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [1.26, 2.1, -0.51, 1.41, 3.9, -1.51, 0.0]
            point.time_from_start = rospy.Duration(2.0)
            # Execute
            self.op.jointTrajectory('Position over tray', names, point)
        else:
            # Move right
            # Create a joint trajectory message and fill with values
            # Create names
            names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # Create points
            point = JointTrajectoryPoint()
            point.positions = [1.26, -2.1, -0.51, 4.62, 3.9, -1.51, 0.0]
            point.time_from_start = rospy.Duration(2.0)
            # Execute
            self.op.jointTrajectory('Position over tray', names, point)

    def moveOverInventory(self):
        # Position the arm to hover over the inventory
        # Create a joint trajectory message and fill with values
        # Create names
        names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Create points
        point = JointTrajectoryPoint()
        point.positions = [2.26, 0.0, -1.26, 3.27, 3.52, -1.51, 0.0]
        point.time_from_start = rospy.Duration(2.0)
        # Execute
        self.op.jointTrajectory('Position over tray', names, point)

    def moveOverBelt(self):
        # Position the arm to hover over the belt center
        # Create names
        names = ['elbow_joint', 'linear_arm_actuator_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Create points
        point = JointTrajectoryPoint()
        point.positions = [1.63, 0.0, -0.70, 0, 3.77, -1.51, 0.0]
        point.time_from_start = rospy.Duration(2.0)
        # Execute
        self.op.jointTrajectory('Position over belt', names, point)
        
    def armState(self):
        # Query arm state. In this instance it is either moving or still based upon velocities
        armState = self.op.armStateData
        totalVel = 0
        state = None
        # Check velocities
        desiredPoints = armState.desired
        # Get sum and divide by joint number
        for vel in desiredPoints.velocities:
            # Get absolute value
            totalVel += abs(vel)
        # Check
        if (totalVel / 7 > 0.05):
            # Robot arm is moving
            if self.pastArmState != 'Moving':
                # Print change
                rospy.loginfo("Arm State: Moving")
            state = 'Moving'
        else:
            # Robot arm is still
            if self.pastArmState != 'Still':
                # Print change
                rospy.loginfo("Arm State: Still")
            state = 'Still'
        # Set past arm state for loggin purposes
        self.pastArmState = state
        # Return state
        return state

    def getPose(self, source, target):
        # Get pose of source against target
        transform = self.tf_buffer.lookup_transform(target,
        source, # source frame
        rospy.Time(0), # get the tf at first available time
        rospy.Duration(2.0)) # wait for 1 second
        # Create pose
        pose = PoseStamped()
        pose.header.frame_id = source
        # Transform
        transformedPose = do_transform_pose(pose, transform)
        # Return
        return transformedPose

    def getPoseDifferance(self, pose_1, pose_2):
        # Get the orientation differance in euler angles rpy
        euler_1 = self.eulerFromQuaternion(pose_1)
        euler_2 = self.eulerFromQuaternion(pose_2)
        print euler_1
        print 'Next pose:'
        print euler_2
        euler_1[0] -= euler_2[0]
        euler_1[1] -= euler_2[1]
        euler_1[2] -= euler_2[2]
        # Return
        return euler_1
    
    def eulerFromQuaternion(self, pose):
        # Get euler values from quaternion values
        (roll, pitch, yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        euler = [roll, pitch, yaw]
        # Return
        return euler

    def quaternionFromEuler(self, r, p, y):
        # Get quaternion values from euler values
        quaternion = quaternion_from_euler(r, p, y)
        # Return
        return quaternion

    def gripper(self, state):
        # Set vacumm gripper state
        self.op.vacuumGripper(state)

    def get_normalized_quat(self, quat):
        """
        Return the normalized version of the provided geometry_msgs/Quaternion
        object.
        """
        quat_out = Quaternion
        magn = self.get_quat_magn(quat)

        # devide each one of the coordinates with the quaternion magnitude
        for comp in ["x", "y", "z", "w"]:
            exec("quat_out.{c} = quat.{c}".format(c=comp))

        return quat_out

    def get_quat_magn(self, quat):
        """Compute the magnitude of a geometry_msgs/Quaternion object.
        This is defined as the square root of the multiplication of the
        quaternion with its conjugate
        """

        return numpy.sqrt(quat.w ** 2 +
                       quat.x ** 2 +
                       quat.y ** 2 +
                       quat.z ** 2)
