#!/usr/bin/env python
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Quaternion

# Inventory module, handles functions associated with parts inventory
class Inventory:
    # A inventory class
    def __init__(self, operators, sensors):
        # Initialise
        self.op = operators
        self.sensors = sensors
        # We need a tf listener to convert poses into arm reference base
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Build an inventory
        self.inventory = self.buildInventory()
        # Tray inventory
        self.tray = {}
        # Tray list
        self.trayPartList = {}
        # Belt list  
        self.beltPartList = {}

    def buildInventory(self):
        # Build inventory using current data
        invent = {}
        # Get scan data
        while self.sensors.scanShopFloor() == False:
            rospy.sleep(0.5)
        scan = self.sensors.scanShopFloor()
        # Create a parts object
        parts = []        
        # Using logical camera data get parts positions, rotations
        cameraParts = scan['Logical Camera']
        for key, value in cameraParts.iteritems():
            # Ignore agv cameras
            if key == 'logical_camera_5' or key == 'logical_camera_6':
               print "WRONG CAMERA!! " + key
            else:
                 for x in value.models:
                    # Add part
                    part = {}
                    part['Type'] = x.type
                    part['Pose'] = x.pose
                    part['Camera pose'] = value.pose
                    part['Time'] = scan['Time']
                    part['Source'] = key
                    parts.append(part)
        # Add to inventory
        invent['Static'] = parts
        # Add belt parts
        
        # Return
        return invent

    def updateInventory(self):
        # Update specific sections of inventory
        # For now just rebuild
        self.inventory = self.buildInventory()
        pass

    def getPart(self, part):
        # Return the pose of a part
        # Simply scan through inventory and return data for the first one found
        for x in self.inventory['Static']:
            # Check part type
            if x['Type'] == part:
                # Return part data
                return x
            else:
                # Do nothing
                pass
        # No part found
        return None

    def removePart(self, part):
        # Remove part from inventory
        self.inventory['Static'].remove(part)

    def buildTrayInventory(self, camera):
        # Build inventory using current data of tray camera
        invent = {}
        # Get scan data
        while self.sensors.scanShopFloor() == False:
            rospy.sleep(0.5)
        scan = self.sensors.scanShopFloor()
        # Create a parts object
        parts = []        
        # Using logical camera data get parts positions, rotations
        cameraParts = scan['Logical Camera']
        for key, value in cameraParts.iteritems():
            # Ignore all cameras except one over tray
            if key == camera:
                for x in value.models:
                    if x.type == 'agv1' or  x.type == 'kit_tray':
                        # Ignore
                        pass
                    else:
                        # Add part
                        part = {}
                        part['Type'] = x.type
                        part['Pose'] = x.pose
                        part['Camera pose'] = value.pose
                        part['Time'] = scan['Time']
                        part['Source'] = key
                        parts.append(part)
            else:
                print "WRONG CAMERA!!"
        # Add to inventory
        self.tray['Static'] = parts
        # Return
        return self.tray

    def getTrayPart(self, trayInventory, camera):
        # From tray inventory return part which is being held by arm 
        for part in trayInventory['Static']:
            # Transform part pose position into world frame, so we can equally compare with tray list
            partPose = PoseStamped()
            partPose.header.frame_id = 'Inventory Part'
            partPose.pose.position.x = part['Pose'].position.x
            partPose.pose.position.y = part['Pose'].position.y
            partPose.pose.position.z = part['Pose'].position.z
            partPose.pose.orientation.x = part['Pose'].orientation.x
            partPose.pose.orientation.y = part['Pose'].orientation.y
            partPose.pose.orientation.z = part['Pose'].orientation.z
            partPose.pose.orientation.w = part['Pose'].orientation.w      
            # Create transform, this takes the current pose from camera and essentailly sets it to world pose instaead of relative to camera
            transform = self.tf_buffer.lookup_transform('world',
            camera + '_frame', # target frame, we keep this xyz orientation / direction when (0,0,0)
            rospy.Time(0), # get the tf at first available time
            rospy.Duration(2.0)) # wait for 2 seconds
            # Transform
            partPose = do_transform_pose(partPose, transform)

            # Compare with list
            if camera not in self.trayPartList:
                # We have yet to add any parts, so assume any inventory parts are held by arm
                # Return this part
                return part
            else:
                # Camera has a mix of parts fixed to tray i.e placed an in arm (or possibly fallen)
                # Set found to false
                found = False
                # Loop through parts in tray
                for x in self.trayPartList[camera]:
                    # Loop through and return only part which is not listed, some variation due to randomness of camera
                    if (x['Pose'].position.x <= partPose.pose.position.x + 0.05
                        and x['Pose'].position.x >= partPose.pose.position.x - 0.05
                        and x['Pose'].position.y <= partPose.pose.position.y + 0.05
                        and x['Pose'].position.y >= partPose.pose.position.y - 0.05
                        and x['Pose'].position.z <= partPose.pose.position.z + 0.1
                        and x['Pose'].position.z >= partPose.pose.position.z - 0.1):
                        # Part is held within our tray list so ignore
                        print 'part in list, ignore!!!'
                        print 'X: ' + str(partPose.pose.position.x) + ' Tray: ' + str(x['Pose'].position.x)
                        print 'Y: ' + str(partPose.pose.position.y) + ' Tray: ' + str(x['Pose'].position.y)
                        print 'Z: ' + str(partPose.pose.position.z) + ' Tray: ' + str(x['Pose'].position.z)
                        # Set found to true
                        found = True
                        break
                # If no part found
                if found == False:
                    # Part not held within list
                    print 'part not in list, hurrah!!'
                    print 'X: ' + str(partPose.pose.position.x) + ' Tray: ' + str(x['Pose'].position.x)
                    print 'Y: ' + str(partPose.pose.position.y) + ' Tray: ' + str(x['Pose'].position.y)
                    print 'Z: ' + str(partPose.pose.position.z) + ' Tray: ' + str(x['Pose'].position.z)
                    # Return this part
                    return part
                       
                       
    def getFallenTrayPart(self, trayInventory, camera):
        # From tray inventory return part which is has fallen on tray
        for part in trayInventory['Static']:
            # Transform part pose position into world frame, so we can equally compare with tray list
            partPose = PoseStamped()
            partPose.header.frame_id = 'Inventory Part'
            partPose.pose.position.x = part['Pose'].position.x
            partPose.pose.position.y = part['Pose'].position.y
            partPose.pose.position.z = part['Pose'].position.z
            partPose.pose.orientation.x = part['Pose'].orientation.x
            partPose.pose.orientation.y = part['Pose'].orientation.y
            partPose.pose.orientation.z = part['Pose'].orientation.z
            partPose.pose.orientation.w = part['Pose'].orientation.w      
            # Create transform, this takes the current pose from camera and essentailly sets it to world pose instaead of relative to camera
            transform = self.tf_buffer.lookup_transform('world',
            camera + '_frame', # target frame, we keep this xyz orientation / direction when (0,0,0)
            rospy.Time(0), # get the tf at first available time
            rospy.Duration(2.0)) # wait for 2 seconds
            # Transform
            partPose = do_transform_pose(partPose, transform)
            # Compare with list
            if camera not in self.trayPartList:
                # We have yet to add any parts, so assume any inventory parts are held by arm
                # Return this part
                return part
            else:
                # Camera has a mix of parts fixed to tray i.e placed an in arm (or possibly fallen)
                # Set found to false
                found = False
                # Loop through parts in tray
                for x in self.trayPartList[camera]:
                    # Loop through and return only part which is not listed, some variation due to randomness of camera
                    if (x['Pose'].position.x <= partPose.pose.position.x + 0.05
                        and x['Pose'].position.x >= partPose.pose.position.x - 0.05
                        and x['Pose'].position.y <= partPose.pose.position.y + 0.05
                        and x['Pose'].position.y >= partPose.pose.position.y - 0.05
                        and x['Pose'].position.z <= partPose.pose.position.z + 0.1
                        and x['Pose'].position.z >= partPose.pose.position.z - 0.1):
                        # Part is held within our tray list so ignore
                        print 'part in list, ignore!!!'
                        print 'X: ' + str(partPose.pose.position.x) + ' Tray: ' + str(x['Pose'].position.x)
                        print 'Y: ' + str(partPose.pose.position.y) + ' Tray: ' + str(x['Pose'].position.y)
                        print 'Z: ' + str(partPose.pose.position.z) + ' Tray: ' + str(x['Pose'].position.z)
                        # Set found to true
                        found = True
                        break
                # If no part found
                if found == False:
                    # Part not held within list
                    print 'part not in list, hurrah!!'
                    print 'X: ' + str(partPose.pose.position.x) + ' Tray: ' + str(x['Pose'].position.x)
                    print 'Y: ' + str(partPose.pose.position.y) + ' Tray: ' + str(x['Pose'].position.y)
                    print 'Z: ' + str(partPose.pose.position.z) + ' Tray: ' + str(x['Pose'].position.z)
                    # Return this part
                    return part














                        
