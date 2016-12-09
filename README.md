# Final-Project-ME495-Group1
Baxter Control Project - Settlers of Catan

## Summary
ROS package for baxter to place "the robber" in Settlers of Catan

## Substitutions & Modifications to Catan
 - Robber is cube with a red surface on top
 - Board has seven tiles with unique solid colors and a white point in the center for location determination
 - Random number generation is used in place of actual dice rolls (however, dice roll is not implemented instead baxter moves three robber cubes to fixed positions (red green and yellow) after an initial signal of sensing the block once moved into field of view)

## Rules
 - Robber cannot be placed back in the black tile (the robber starting position)
 - Baxter will look at the game board using a pre-set camera and arm configuration to determine initial positions of the game
 - Baxter will generate a random number corresponding to the board.
 - Baxter will move the robber from the current location a random tile on the board

## Documentation
###Names
 - Package Name: Baxter_Catan
 - Launch File: launch/settlers_of_catan.launch
 - Execution File: pickandplace.py (sourced from http://sdk.rethinkrobotics.com/wiki/IK_Pick_and_Place_Demo_-_Code_Walkthrough)
 - opencv1.py (Mario Sebasco helped us get started)
 - ik_service_client2.py
### Dependencies
 - tf
 - visualization_msgs
 - sensor_msgs
 - geometry_msgs
 - std_msgs
 - random
 - baxter_interface
 - cv_bridge
 - cv2
 
### Active Nodes
##### baxter_movement
<p>Contained within ik_service_client2.py, it subscribes to opencv/centerofobject to get the center of the block. It then uses that information to generate the static x and y positions of the rest of the board. Locations of random tiles are used to create movement using the built-in Baxter service rospy.ServiceProxy(ns, SolvePositionIK) to determine the joint angles needed for the movement. This movement is then implemented by baxter_interface.Limb('right').move_to_joint_positions(limb_joints) where limb joints is the specified joint angles</p>

<p>A number of functions are defined within ik_service_client2.py to isolate different parts of the movement:</p>

<ol>
 <li>main():
  <ul> 
   <li> Main run function for ik_service_client2.py.</li>
   <li> Initializes 'baxter_movement' node</li>
   <li> Starts up the Baxter Robot and calls 'BaxterMovement(p0,q)' to move Baxter into the initial position configuration</li>
   <li> Subscribes to the "opencv/center_of_object" topic published by the 'listener' node for the block position and pushes the output to the callback function 'imagecb(data)' for further processing</li>
  </ul>
 </li>
 <li>BaxterMovement(p,q):
  <ul> 
   <li> Calls Baxter built-in service 'SolvePositionIK' to determine the joint angles for a position 'p' and a hard-coded quaternion 'q' to keep the gripper holding the block facing downward</li> 
   <li> Calls Baxter built-in function 'SolvePositionIKRequest()' to aid movement determination</li> 
   <li> Calls 'move_to_joint_positions(limb_joints)' to effect the movement to the new joint angles specified by 'SolvePositionIK'</li> 
  </ul>
 </li>
 <li>imagecb(data):
  <ul> 
   <li> Takes the data from the "opencv/center_of_object" topic to get the x and y coordinates of the center block</li> 
   <li> From that initial data, it generates the positions of the rest of the board </li> 
   <li> Uses 'BaxterMovement' to effect movement to different tiles</li> 
  </ul>
</ol>
##### open_cv1.py
<p>
</p>

## Video
 - Three stacked blocks (https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/video/video_link.md)
