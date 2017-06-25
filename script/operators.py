#!/usr/bin/env python
import rospy
import numpy
import time

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
from osrf_gear.msg import LogicalCameraImage
from osrf_gear.msg import Order
from osrf_gear.msg import Proximity
from osrf_gear.msg import VacuumGripperState
from osrf_gear.srv import GetMaterialLocations
from osrf_gear.srv import VacuumGripperControl
from osrf_gear.srv import  AGVControl

class Operators:
    # An operator class
    def __init__(self):
        # Initialise
        rospy.loginfo('Initialising operators')
        # Declare variables
        self.currentScore = None
        self.competitionStates = None
        self.currentOrders = []
        self.jointStateData = None
        self.gripperStateData = None
        self.armStateData = None
        self.proximitySensorData = None
        self.breakBeamData = {}
        self.logicalCameraData = {}       
        self.laserProfilerData = None
        self.agvStateData = ['Null', 'Null']
        # Declare rate variable
        self.pastJointStateTime = rospy.get_time()
        self.pastArmStateTime = rospy.get_time()
        self.pastLogicalCameraTime = rospy.get_time()
        self.pastLaserProfileTime = rospy.get_time()
        self.pastAGV1StateTime = rospy.get_time()
        # Initialise publishers
        self.jointTrajectoryPublisher = rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
        self.visual_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.pose_publisher = rospy.Publisher('pose_marker', PoseStamped, queue_size=5)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        # Initialise subscribers
        self.currentScoreSubscriber = rospy.Subscriber("/ariac/current_score", Float32, self.currentScore)
        self.competitionStateSubscriber = rospy.Subscriber("/ariac/competition_states", String, self.competitionStates)
        self.ordersSubscriber = rospy.Subscriber("/ariac/orders", Order, self.orders)
        self.jointStateSubscriber = rospy.Subscriber("/ariac/joint_states", JointState, self.jointState)
        self.proximitySensorSubscriber = rospy.Subscriber("/ariac/proximity_sensor_1_change", Proximity, self.proximitySensor)
        self.break_beam_subscriber_1 = rospy.Subscriber("/ariac/break_beam_1_change", Proximity, self.breakBeam, 'break_beam_1_change')
        self.break_beam_subscriber_2 = rospy.Subscriber("/ariac/break_beam_2_change", Proximity, self.breakBeam, 'break_beam_2_change')
        self.logicalCameraSubscriber_1 = rospy.Subscriber("/ariac/logical_camera_1", LogicalCameraImage, self.logicalCamera, 'logical_camera_1')
        self.logicalCameraSubscriber_2 = rospy.Subscriber("/ariac/logical_camera_2", LogicalCameraImage, self.logicalCamera, 'logical_camera_2')
        self.logicalCameraSubscriber_3 = rospy.Subscriber("/ariac/logical_camera_3", LogicalCameraImage, self.logicalCamera, 'logical_camera_3')
        self.logicalCameraSubscriber_4 = rospy.Subscriber("/ariac/logical_camera_4", LogicalCameraImage, self.logicalCamera, 'logical_camera_4')
        self.logicalCameraSubscriber_5 = rospy.Subscriber("/ariac/logical_camera_5", LogicalCameraImage, self.logicalCamera, 'logical_camera_5')
        self.logicalCameraSubscriber_6 = rospy.Subscriber("/ariac/logical_camera_6", LogicalCameraImage, self.logicalCamera, 'logical_camera_6')
        self.laserProfilerSubscriber_7 = rospy.Subscriber("/ariac/laser_profiler_1", LaserScan, self.laserProfiler)
        self.armStateSubscriber = rospy.Subscriber("/ariac/arm/state", JointTrajectoryControllerState, self.armState)
        self.gripperStateSubscriber = rospy.Subscriber("/ariac/gripper/state", VacuumGripperState, self.vacuumGripperState)
        self.agv1StateSubscriber = rospy.Subscriber("/ariac/agv1/state", String, self.agv1State)
        self.agv2StateSubscriber = rospy.Subscriber("/ariac/agv2/state", String, self.agv2State)
        # Initialise services
        rospy.wait_for_service('/ariac/material_locations')
        self.materialLocationsService1 = rospy.ServiceProxy('/ariac/material_locations',  GetMaterialLocations)
        rospy.wait_for_service('/ariac/gripper/control')
        self.gripperService = rospy.ServiceProxy('/ariac/gripper/control',  VacuumGripperControl)
        rospy.wait_for_service('/ariac/agv1')
        self.agv1Service = rospy.ServiceProxy('/ariac/agv1', AGVControl)
        rospy.wait_for_service('/ariac/agv2')
        self.agv2Service = rospy.ServiceProxy('/ariac/agv2', AGVControl)
        rospy.wait_for_service('/ariac/end_competition')
        self.endCompetitionService = rospy.ServiceProxy('/ariac/end_competition', Trigger)
       
    def currentScore(self, msg):
        # Get current score
        # Print if changed
        if self.currentScore != msg:
            rospy.loginfo('Current Score: ' + str(msg))
        # Update
        self.currentScore = msg
        
    def competitionStates(self, msg):
        # Get competition states
        # Print if changed
        if self.competitionStates != 'done' and msg.data == 'done':
            rospy.loginfo('Competition Ended')
        # Update
        self.competitionStates = msg.data

    def endCompetition(self):
        # Simply end compeition
        resp = self.endCompetitionService()
         # Check if successful
        if (resp.success != True):
            rospy.logerr("Failed to end the competition: " + resp.message);
        else:
            rospy.loginfo("Competition has now been ended!")

    def orders(self, msg):
        # Get orders
        # Print if changed
        rospy.loginfo('Recieved Order: ' + str(msg))  
        # Update with new order
        self.currentOrders.append(msg)

    def removeOrder(self, orderID):
        # Loop through orders and remove matching ID
         for order in self.currentOrders:
            # Check name
            if order.order_id == orderID:
                # Remove
                self.currentOrders.remove(order)

    def jointState(self, msg):
        # Get joint state
        # Log periodically
        now = rospy.get_time()
        if self.pastJointStateTime + 1.0 < now:
            # Log joint state
            #rospy.loginfo("Joint States: " + str(msg))
            # Set new past time
            self.pastJointStateTime = rospy.get_time()
            # Update
            self.jointStateData = msg
        
    def proximitySensor(self, msg):
        # Get proximity sensor
        # Print if object in proximity
        if msg.object_detected:
            rospy.loginfo('Proximity Sensor Triggered')
        # Update
        self.proximitySensorData = msg

    def breakBeam(self, msg, source):
        # Get break beam
        # Print if object breaks beam
        if msg.object_detected:
            rospy.loginfo('Break Beam Triggered')
        # Update
        if msg is not None:
            # Check if message for objected detected is true or false
            if msg.object_detected == true:
                # Object detected
                self.breakBeamData[str(source)] = {}
                self.breakBeamData[str(source)]['Enter Time'] = 0
                self.breakBeamData[str(source)]['Leave Time'] = 0
            else:
                # Object not detected
                if self.breakBeamData[str(source)] == None:
                    # Do nothing
                    pass
                else:
                    # Check if enter time is there for all objects
                    for obj in self.breakBeamData[str(source)]:
                        # Check leave times
                        if obj['Leave Time'] == 0:
                            # Set leave time
                            obj['Leave Time'] = rospy.get_time()
                        else:
                            # Ignore
                            pass

    def logicalCamera(self, msg, source):
        # Log periodically
        now = rospy.get_time()
        if self.pastLogicalCameraTime + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTime = rospy.get_time()
        # Update
        if msg is not None:          
            self.logicalCameraData[str(source)] = msg

    def laserProfiler(self, msg):
        # Get laser profiler
        # Log periodically
        now = rospy.get_time()
        # Loop through range and check if we have a value
        for x in msg.ranges:
            if isinstance(x, Float32):
                # Value present
                 if self.pastLaserProfileTime + 5.0 < now:
                    # Log laser profile
                    #rospy.loginfo("Laser Profiler Senses: " + str(x) + " Value")
                    # Set new past time
                    self.pastLaserProfileTime = rospy.get_time()
                    break
        # Update
        self.laserProfilerData = msg

    def materialLocations1(self, msg):
        # Get material locations for type
        return self.materialLocationsService(msg)

    def vacuumGripper(self, msg):
        # Command the gripper
        self.gripperService(msg)

    def vacuumGripperState(self, msg):
        # Get gripper state
        self.gripperStateData = msg

    def jointTrajectory(self, ID, jointNames, points):
        # Command a joint trajectory of arm
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = ID
        msg.joint_names = jointNames
        msg.points = []
        msg.points.append(points)
        # Publish
        self.jointTrajectoryPublisher.publish(msg)

    def armState(self, msg):
        # Get arm state
        # Log periodically
        now = rospy.get_time()
        if self.pastArmStateTime + 1.0 < now:
            # Log arm state
            #rospy.loginfo("Arm State: " + str(msg))
            # Set new past time
            self.pastArmStateTime = rospy.get_time()
        # Update
        self.armStateData = msg

    def submitAGV(self, agv, msg):
        # Submit AGV for evaluation
        if agv == 0:
            # Submit first AGV i.e left
            self.agv1Service(msg)
        else:
            # Submit second AGV i.e right
            self.agv2Service(msg)

    def agv1State(self, msg):
        # Log periodically
        now = rospy.get_time()
        if self.pastAGV1StateTime + 1.0 < now:
            # Log arm state
            #rospy.loginfo("AGV 1 State: " + str(msg))
            # Set new past time
            self.pastAGV1StateTime = rospy.get_time()
        # Set agv 1 state
        self.agvStateData[0] = msg

    def agv2State(self, msg):
        # Set agv 2 state
        self.agvStateData[1] = msg

    def getAGVState(self, AGV):
        # Get the state of AGV using provided tag
        if AGV == 1:
            return self.agvStateData[0]
        elif AGV == 2:
            return self.agvStateData[1]
        
    def visualisePlan(self, robot, plan):
        # Visualise a plan
        print "============ Visualizing plan"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        print "============ Waiting while plan is visualized (again)..."

    def marker(self, pose):
        # Create a marker
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "Factory"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # Publish
        self.visual_publisher.publish(marker)

    def poseMarker(self, poseStamped):
        # Publish pose
        self.pose_publisher.publish(poseStamped)



        
