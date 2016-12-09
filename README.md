# Final-Project-ME495-Group1
Baxter Control Project - Settlers of Catan

## Summary
ROS package for baxter to place "the robber" in Settlers of Catan

## Substitutions & Modifications to Catan
 - Robber is cube with a red surface on top
 - Board has seven tiles with unique solid colors and a white point in the center for location determination https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/tree/master/images
 - Random number generation is used in place of actual dice rolls (however, dice roll is not implemented instead baxter moves three robber cubes to fixed positions (red green and yellow) after an initial signal of sensing the block once moved into field of view)

## Rules
 - Robber cannot be placed back in the black tile (the robber starting position)
 - Baxter will look at the game board using a pre-set camera and arm configuration to determine initial positions of the game
 - Baxter will generate a random number corresponding to the board.
 - Baxter will move the robber from the current location a random tile on the board

## Documentation
###Names
 - Package Name: Baxter_Catan
 - Launch File: launch/settlers_of_catan.launch https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/launch/settlers_of_catan.launch
 - pickandplace.py https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/src/pickandplace.py (inspired by http://sdk.rethinkrobotics.com/wiki/IK_Pick_and_Place_Demo_-_Code_Walkthrough)
 - Execution File: opencv1.py https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/src/opencv1.py (Mario Sebasco helped us get started)
 - Execution File: ik_service_client2.py https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/src/ik_service_client2.py
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
 
### Active Nodes at Presentation
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
  </li>
</ol>
##### listener
<p> Contained within opencv1.py, this node subscribes to message from Baxter "/cameras/right_hand_camera/image", and then uses bridge to collect color data from the camera. This color data is then converted into an hsv for analysis. This color data is analyzed to find the block which is red and then publishes the center location of the block on the topic "opencv/center_of_object" for use by 'baxter_movement'
</p>
<p>A number of functions are defined within opencv1.py:</p>

<ol>
 <li>listener():
  <ul> 
   <li> Main run function for opencv1.py</li>
   <li> Initializes 'listener' node</li>
   <li> Gets images from Baxter's right hand camera</li>
   <li> Subscribes to the "opencv/center_of_object" topic published by the 'listener' node for the block position and pushes the output to the callback function 'imagecb(data)' for further processing</li>
  </ul>
 </li>
 <li>imagecb(data):
  <ul> 
   <li> Converts data to HSV data type with cv2.cvtColor function</li> 
   <li> Defines upper and lower ranges for expected values of the red block</li> 
   <li> Contains upper and lower vallues for blue and yellow that are not used</li> 
   <li> Defines a 'mask' for values that are captured within the 'red' defined within range</li> 
   <li> Identifies the 'x' and 'y' coordinates of the center of the mask (defined by the red block)</li> 
   <li> Publishes the 'x' and 'y' coordinates to the "opencv/center_of_object" topic </li> 
  </ul>
 </li>
</ol>
## Video for Presentation
 - Three stacked blocks (https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/video/video_link.md)

## Function and Approaches not used in the Presentation
##### pickandplace.py
<p> Now successfully runs. Uses a dot in the center of surfaces to provide positions. Example: https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/images/center_point.png </p>
##### Using AR tags
<p> Earlier implementation attempted use of AR tags for board identification. However, this attempt was abandoned when not all of the tags could be identified in one camera image. </p>
##### Using a larger board
<p> One stretch goal was to use a large Catan board https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/images/Catan_Board_Large.JPG </p>
