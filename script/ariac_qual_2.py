#!/usr/bin/env python
import rospy
import operators
import orders
import sensors
import inventory
import armController
import scene
from std_srvs.srv import Trigger


# Start the competition by waiting for and then calling the start ROS Service.
def Start():
    # Wait for service to be active
    rospy.loginfo("Waiting for the competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    start_service = rospy.ServiceProxy('/ariac/start_competition', Trigger)
    # Success
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")
    # Call start
    resp = start_service()
    # Check if successful
    if (resp.success != True):
        rospy.logerr("Failed to start the competition: " + resp.message);
    else:
        rospy.loginfo("Competition started!")

def BuildScene():
    # Build planning scene in which our arm operates, set world as root link
    planningScene = scene.PlanningSceneInterface("world")
    # Clear planning scene
    planningScene.clear()
    # add a box to represent zone of bins
    planningScene.addBox("bins", 1.0, 5.0, 0.1, -0.3, -0.1675, 0.66)
    # Set colour
    planningScene.setColor("bins", 124, 12, 255)
    # add a box to represent the zone of cage top
    planningScene.addBox("cage", 1.0, 5.0, 0.05, -0.5, -0.1675, 2.2)
    # Set colour
    planningScene.setColor("cage", 124, 12, 0)
    # add a box to represent the zone of conveyor belt
    planningScene.addBox("belt", 0.75, 5.0, 0.05, 1.24, 0.15, 0.85)
    # Set colour
    planningScene.setColor("belt", 124, 12, 0)

def FloorShopManager():
    #  Declare variables
    op = operators.Operators()
    sen = sensors.Sensors(op)
    arm = armController.ArmController(op)
    inv = inventory.Inventory(op, sen) 
    # Loop through process of completing orders on the shop floor
    while not rospy.is_shutdown():
        # Listen for orders
        if len(op.currentOrders) > 0:
            # There are orders to be processed
            order = orders.Orders(op.currentOrders[0])
            agv = 'agv1'
            tray = 'Left'
            # loop through each kit and process
            for kit in order.kits:
                # Get type of parts we need to add to tray
                for part in kit['Objects']:
                    # Set procede to false
                    proceed = False
                    # Check whether this part has successfully being placed on tray and should procede
                    while proceed == False:
                        # Return part location from inventory
                        inventoryPart = None
                        # Set desired pose of part in tray
                        desiredPose = part.pose
                        # Update inventory
                        inv.updateInventory()
                        # Wait for step to complete
                        while inventoryPart is None:
                            # Loop and wait for part to be found
                            inventoryPart = inv.getPart(part.type)
                            # Get part pose
                            #rospy.loginfo("Part: " + str(inventoryPart))
                            #rospy.loginfo("Desired Pose: " + str(desiredPose))
                            # Check if found, if not wait and try again
                            if inventoryPart is None:
                                # Wait
                                rospy.sleep(1.0)
                                # Scan inventory and update (this is really only for later when new parts
                                # are being actively placed on belt)
                                inv.updateInventory()
                            # Check belt
                            
                        print "=========== Transfer Part: " + str(inventoryPart['Type'])
                        print "=========== Transfer Source: " + str(inventoryPart['Source'])
                        while op.gripperStateData.attached != True:
                            # Move over inventory
                            arm.moveOverInventory()
                            # Wait
                            rospy.sleep(1.0)
                            # Move arm to above part position
                            print 'Move arm to part'
                            pose = arm.transformPose(inventoryPart['Pose'], [0,0,0.2], [0,0,0,0], inventoryPart['Source'] + '_frame')
                            arm.poseTarget(pose)
                            #rospy.loginfo("Arm Pose for part" + str(pose))
                            # Plan
                            arm.planPose()
                            # Execute plan
                            arm.executePlan()
                            # Wait
                            rospy.sleep(1.5)
                            # Move arm into contact with part
                            pose = arm.transformPose(inventoryPart['Pose'], [0.0,0,0.02], [0,0,0,0], inventoryPart['Source'] + '_frame')
                            arm.poseTarget(pose)
                            # Plan
                            arm.planPose()
                            # Execute plan
                            arm.executePlan()
                            # Wait
                            rospy.sleep(0.5)
                            # Activate gripper first attempt
                            arm.gripper(True)
                            # Check
                            if op.gripperStateData.attached != True:
                                # Gripper failed
                                print "Gripper attempt failed!"
                                # Move arm into contact with part
                                pose = arm.transformPose(inventoryPart['Pose'], [0.0,0,0.01], [0,0,0,0], inventoryPart['Source'] + '_frame')
                                arm.poseTarget(pose)
                                arm.planPose()
                                arm.executePlan()
                                # Wait
                                rospy.sleep(0.5)
                                # Set count
                                count = 0
                                # Activate gripper in loop for 1.0 second
                                while op.gripperStateData.attached != True and count < 5:
                                    arm.gripper(True)
                                    # Update count and wait
                                    count += 1
                                    # Wait
                                    rospy.sleep(0.2)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                            # Wait
                            rospy.sleep(1.0)
                            # Return arm to holding position
                            arm.handlePart()
                            # Wait 
                            rospy.sleep(1.5)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                            if op.gripperStateData.attached != True:
                                # Gripper failed
                                print "Gripper failed part has fallen!"
                                # Reset inventory part and find a new one
                                inventoryPart = None
                                # are being actively placed on belt)
                                inv.updateInventory()   
                                # Find part
                                while inventoryPart is None:
                                    # Loop and wait for part to be found
                                    inventoryPart = inv.getPart(part.type)
                                    # Check if found, if not wait and try again
                                    if inventoryPart is None:
                                        # Wait
                                        rospy.sleep(1.0)
                                        # Scan inventory and update (this is really only for later when new parts
                                        # are being actively placed on belt)
                                        inv.updateInventory()                        
                        # Move part to desired tray location
                        arm.moveToTray(tray)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Move part to desired tray location
                        arm.moveOverTray(tray)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Get tray pose
                        trayPose = arm.trayPose(desiredPose, 'logical_camera_5_kit_tray_1_frame', inv)
                        trayPose.pose.position.z += 0.3
                        # Place part in correct position on tray according to order
                        arm.poseTarget(trayPose)
                        arm.planPose()
                        arm.executePlan()
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Check whether we should procede
                        if op.gripperStateData.attached != True:
                            # Part has fallen off
                            print "Gripper is empty, Oh My!!"
                            proceed = False
                        else:
                            # Successful
                            proceed = True



                        # If our part fell off, retrieve from tray and place again
                        if (proceed == False):
                            # Move part to desired tray location
                            arm.moveOverTray(tray)
                            # Wait 
                            rospy.sleep(0.5)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                            # Attempt to pick up part, first get part on tray
                            fallenPart = arm.getFallenTrayPart('logical_camera_5_kit_tray_1_frame', inv)
                            # Offset z axis
                            fallenPart.pose.position.z += 0.02
                            arm.poseTarget(fallenPart)
                            # Plan
                            arm.planPose()
                            # Execute plan
                            arm.executePlan()
                            # Wait
                            rospy.sleep(1.5)
                            # Activate gripper first attempt
                            arm.gripper(True)
                            # Check
                            if op.gripperStateData.attached != True:
                                # Gripper failed
                                print "Gripper attempt failed!"
                                # Move arm into contact with part
                                fallenPart.pose.position.z -= 0.01
                                arm.poseTarget(fallenPart)
                                arm.planPose()
                                arm.executePlan()
                                # Wait
                                rospy.sleep(0.5)
                                # Set count
                                count = 0
                                # Activate gripper in loop for 1.0 second
                                while op.gripperStateData.attached != True and count < 5:
                                    arm.gripper(True)
                                    # Update count and wait
                                    count += 1
                                    # Wait
                                    rospy.sleep(0.2)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass                       
                            # Move part to desired tray location
                            arm.moveOverTray(tray)
                            # Wait 
                            rospy.sleep(0.5)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                             # Get tray pose
                            trayPose = arm.trayPose(desiredPose, 'logical_camera_5_kit_tray_1_frame', inv)
                            trayPose.pose.position.z += 0.3
                            # Place part in correct position on tray according to order
                            arm.poseTarget(trayPose)
                            arm.planPose()
                            arm.executePlan()
                            # Wait 
                            rospy.sleep(0.5)
                            while arm.armState() == 'Moving':
                                # Hold here
                                pass
                            # Check whether we should procede
                            if op.gripperStateData.attached != True:
                                # Part has fallen off
                                print "Gripper is empty, Oh My!!"
                                proceed = False
                            else:
                                # Successful
                                proceed = True



                            
                        # Inactivate gripper
                        arm.gripper(False)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Remove part from inventory
                        #inv.removePart(inventoryPart)
                        # Retract arm
                        arm.moveToTray(tray)
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Return arm to holding position
                        arm.handlePart()
                        # Wait 
                        rospy.sleep(0.5)
                        while arm.armState() == 'Moving':
                            # Hold here
                            pass
                        # Move over inventory
                        arm.moveOverInventory()
                        # Wait for some reason otherwise moveit screws up
                        rospy.sleep(1.0)
                        # Update inventory
                        inv.buildInventory()        
                # Send AGV away
                print "Sending AGV away,..."
                # Set current agv state
                state = op.getAGVState(1)
                # Check
                if agv == 'agv1':
                    # Submit 1st agv
                    op.submitAGV(0, 'order_0')
                    rospy.sleep(0.5)
                    # Wait for return
                    while state != op.getAGVState(1):
                        # Hold here and wait for agv to return
                        if state == op.getAGVState(1):
                            print "AGV 1 back, restart!,.."
                            break
                    # Remove order
                    op.removeOrder(order.ID)
                else:
                    # Submit 2nd agv
                    op.submitAGV(1, 'order_0')
                    # Wait for return
                    while state != op.getAGVState(2):
                        # Hold here and wait for agv to return
                        if state == op.getAGVState(2):
                            print "AGV 2 back, restart!,.."
                            break
                    # Remove order
                    op.removeOrder(order.ID)
                print "Next kit?"
        else:
            # Do nothing and wait
            print "No Orders!!"
            op.endCompetition()
            rospy.sleep(2.5)



 

if __name__ == '__main__':
    try:
        rospy.init_node('ariac_qual_1')
        # Start competition
        Start()
        # Build scene
        BuildScene()
        # Call floor shop manager
        FloorShopManager()
        # Spin up ROS
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.15 -z 0.1 -R 0 -P 0 -Y 0 -file `catkin_find osrf_gear --share`/models/piston_rod_part/model.sdf -reference_frame agv1::kit_tray::kit_tray::tray -model piston_rod_part_1


