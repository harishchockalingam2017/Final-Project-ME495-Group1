# Final-Project-ME495-Group1
Baxter Control Project
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
### Nodes
##### ik_service_cli
<p>ik_service_client2.py subscribes to opencv/centerofobject to get the center of the block. It then uses that information to generate the static positions of the rest of the board. Locations to random tiles are used to create movement using.</p>


## Video
 - Three stacked blocks (https://github.com/harishchockalingam2017/Final-Project-ME495-Group1/blob/master/video/video_link.md)
