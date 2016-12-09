#!/usr/bin/env python
import argparse
import struct
import sys
import rospy

import copy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, Empty

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from ar_track_alvar_msgs.msg import AlvarMarkers

from random import randint

from baxter_interface import CHECK_VERSION
import baxter_interface
from baxter_core_msgs.msg import EndpointState

#cv2
import cv2
from cv_bridge import CvBridge, CvBridgeError

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self.list_of_poses = []
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        while True:
            try:
                resp = self._iksvc(ikreq)
                break
            except (rospy.ServiceException, rospy.ROSException), e:
                pass
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                                    ikreq.SEED_USER: 'User Provided Seed',
                                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                                    }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format((seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
                rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
                return False

        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def callback(data, args):
    # if there was an empty array, why you even callin me brah
    if len(data.markers) == 0: return
    pnp = args[0]
    #we know there should be 7 (6 tiles and a robber cube)
    if len(pnp.list_of_poses) != 7:
        for i in range(0, len(data.markers)):
            curr_Pose = data.markers[i].pose.pose
            neverIn = True
            for j in range(0, pnp.list_of_poses):
                if curr_Pose.position.z == pnp.list_of_poses[j].position.z:
                    neverIn = False # it was in
            if neverIn:
            	pnp.list_of_poses.append(curr_Pose)

    #do we have enough poses now?
    if len(pnp.list_of_poses) > 7:
        # whaaaaat so do we have duplicates then?!?!?!?
        unique = [pnp.list_of_poses[0]]
        for i in range(0, len(pnp.list_of_poses)):
            cur_pnp_pose = pnp.list_of_poses[i]
            duplicate = False
            for j in range(0, len(unique)):
                if unique[j] == cur_pnp_pose:
                    #duplicate value
                    duplicate = True
                    break
            if duplicate:
                continue
            unique.append(cur_pnp_pose)
        # save value
        pnp.list_of_poses = unique
        
    # how about now? enough poses?
    if len(pnp.list_of_poses) != 7: return
    # normal stuff we were doin before
    rate = rospy.Rate(10)
    # pull cube out of board qr
    cube_pos = Pose()
    cube_pos.position = Point(0, 0, 0)
    marker_poses = []
    for i in range(0, len(pnp.list_of_poses)):
        curr_Pose = pnp.list_of_poses[i]
        #check z value for cube
        if curr_Pose.position.z > cube_pos.position.z:
            marker_poses.append(cube_pos)
            cube_pos = curr_Pose
        else:
            marker_poses.append(curr_Pose)

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779,y=0.999649402929,z=0.00737916180073,w=0.00486450832011)
    block_poses = list()
    for i in range(0, len(marker_poses)):
        block_poses.append(Pose(position=Point(x=marker_poses[i].position.x, y=marker_poses[i].position.x, z=cube_pos.position.z),orientation=marker_poses[i].orientation))

    curr_loc = cube_pos
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(curr_loc)
        print("\nPlacing...")
        curr_loc = block_poses[randint(0, smaller)]
        pnp.place(curr_loc)
        rate.sleep()

# openCV attempt
def WaitForImage(bridge):
    #image subscriber
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, GetBoardGraph, callback_args=(bridge,))
    #ImagTrackBar()
    try:
        rospy.spin()

def GetBoardPositions(data, args):
	bridge = args[0]
    # original image
    try:
        imgOriginal = bridge.imgmsg_to_cv2(data, "bgr8")
    # color ranges
    #colors_of_tiles = [[], [], [], [], [], []]
    #for i in range(0, 6):
    #	h_low,s_low,v_low,h_hi,s_hi,v_hi = Trackbar("MyImage",imgOriginal)
    #	lower = np.array([h_low,s_low,v_low])
    #	upper = np.array([h_hi,s_hi,v_hi])
    #	colors_of_tiles[i].append(lower)
    #	colors_of_tiles[i].append(upper)
    #find center points
    centerPoints = getPointsFromImage(imgOriginal)
    ####call ik stuff here ####
    center = centerPoints[3]
    centerPoints.Pop(3)

    #init baxter grip
    baxterright = baxter_interface.Gripper('right')
    baxterright.calibrate()

    #start by moving cube from center to somewhere
    last_pos = centerPoints[randint(0, len(centerPoints)-1)]
    q = Quaternion(x=-0.0177807149532, y=0.999274143222, z=-0.00996636325311, w=-0.0321823626761)
    
    # open gripper
    x = center.x
    y = center.y    
    xfac = 530.0
    yfac = -464.0
    xp = x/xfac+0.01
    yp = y/yfac
    p1 = Point(x=xp,y=yp,z=-0.1851726872428) # old pos down
    p2 = Point(x=xp,y=yp,z=-0.0351726872428) # old pos up
    BaxterMovement(p2,q)
    # open gripper
    baxterright.open()
    # go to prev pose 
    BaxterMovement(p1,q)
    # grab block
    baxterright.close()
    BaxterMovement(p2,q)
    
    # go to new pose with block
    x = last_pos.x
    y = last_pos.y    
    xfac = 530.0
    yfac = -464.0
    xp = x/xfac+0.01
    yp = y/yfac
    p3 = Point(x=xp,y=yp,z=-0.0351726872428) #new pose up
    p4 = Point(x=xp,y=yp,z=-0.1851726872428) #down

    BaxterMovement(p3,q)
    BaxterMovement(p4,q)
    # open
    baxterright.open()
    BaxterMovement(p3,q)

    # go back to origin
    BaxterMovement(p2, q)

    while True:
        x = last_pos.x
        y = last_pos.y    
        xfac = 530.0
        yfac = -464.0
        xp = x/xfac+0.01
        yp = y/yfac
        p1 = Point(x=xp,y=yp,z=-0.1851726872428) # old pos down
        # open gripper
        baxterright.calibrate()
        baxterright.open()
        # go to prev pose 
        BaxterMovement(p1,q)
        # grab block
        baxterright.close()
        p2 = Point(x=xp,y=yp,z=-0.0351726872428) # old pos up
        BaxterMovement(p2,q)
        
        # go to new pose with block
        cur_pos = centerPoints[randint(0, len(centerPoints)-1)]
        last_pos = cur_pos
        x = cur_pos.x
        y = cur_pos.y    
        xfac = 530.0
        yfac = -464.0
        xp = x/xfac+0.01
        yp = y/yfac
        p3 = Point(x=xp,y=yp,z=-0.0351726872428) #new pose up
        p4 = Point(x=xp,y=yp,z=-0.1851726872428) #down

        BaxterMovement(p3,q)
        BaxterMovement(p4,q)
        # open
        baxterright.open()
        BaxterMovement(p3,q)

        # go back to origin
        x = center.x
        y = center.y    
        xfac = 530.0
        yfac = -464.0
        xp = x/xfac+0.01
        yp = y/yfac
        p5 = Point(x=xp,y=yp,z=-0.0351726872428) #back to center up
        BaxterMovement(p5, q)

def getPointsFromImage(img):
    img = cv2.imread(filename)
    edges = cv2.Canny(img,10,100,apertureSize=3)
    dst = cv2.cornerHarris(edges,16,5,.1)
    black_and_white = np.zeros(img.shape, np.uint8)
    black_and_white[dst>0.01*dst.max()]=[255,255,255]
    gray = cv2.cvtColor(black_and_white, cv2.COLOR_BGR2GRAY)
    cnt = cv2.findContours(gray, 1, 2)[0]
    for c in cnt:
        x,y,w,h = cv2.boundingRect(c)
        point = (x+int(.5*w),y+int(.5*h))
        points.append(point)
    #sort points by y with thresh
    y_thresh = 2.0
    sorted_points_by_y = []
    for p in range(0, len(points)):
        point = points[p]
        # check against already sorted
        b = False
        for q in range(0, len(sorted_points_by_y)):
            for r in range(0, len(sorted_points_by_y[q])):
                if np.abs(float(point[1])-float(sorted_points_by_y[q][r][1])) < y_thresh:
                    sorted_points_by_y[q].append(point)
                    b = True
                    break
            if b:
                break
        #if still not added, make new category and add
        if not b:
            sorted_points_by_y.append([point])
    points = []
    for s in range(0, len(sorted_points_by_y)):
        if len(sorted_points_by_y[s]) < 2:
            np.delete( sorted_points_by_y, s)
            --s
        else:
            for p in range( 0, len(sorted_points_by_y[s])):
                points.append(sorted_points_by_y[s][p])
    #old fasion sort of points by x value
    sorted_points_tmp = [points[0]]
    for p in range(1, len(points)):
        for q in range(0, len(sorted_points_tmp)):
            if points[p][0] < sorted_points_tmp[q][0]:
                sorted_points_tmp.insert(q, points[p])
                break
            elif q == len(sorted_points_tmp)-1: # append if this was last item
                sorted_points_tmp.append(points[p])
                break
    points = sorted_points_tmp
    print points
    #sort points by x with thresh
    x_thresh = 25
    sorted_points_by_x = []
    for p in range(0, len(points)):
        point = points[p]
        # check against already sorted
        b = False
        for q in range(0, len(sorted_points_by_x)):
            for r in range(0, len(sorted_points_by_x[q])):
                if np.abs(np.abs(float(point[0]))-np.abs(float(sorted_points_by_x[q][r][0]))) < x_thresh:
                    sorted_points_by_x[q].append(point)
                    b = True
                    break
            if b:
                break
        #if still not added, make new category and add
        if not b:
            sorted_points_by_x.append([point])
    points = []
    even = True
    for s in range(0, len(sorted_points_by_x)):
        if len(sorted_points_by_x[s]) < 2:
            np.delete( sorted_points_by_x, s)
            --s
        elif even:
            np.delete( sorted_points_by_x, s)
            --s
            even = False
        else:
            even = True
            for p in range( 0, len(sorted_points_by_x[s])):
                points.append([sorted_points_by_x[s][p][0], sorted_points_by_x[s][p][1]])
    return points

#trackbar stuff
def ImagTrackBar():
    win = cv2.namedWindow("MyImage")
    name = "MyImage"
    cv2.createTrackbar('H_low',name,0,255,tb.nothing)
    cv2.createTrackbar('S_low',name,164,255,tb.nothing)
    cv2.createTrackbar('V_low',name,80,255,tb.nothing)
    cv2.createTrackbar('H_high',name,180,255,tb.nothing)
    cv2.createTrackbar('S_high',name,220,255,tb.nothing)
    cv2.createTrackbar('V_high',name,255,255,tb.nothing)

def trackbar(name, image):
    # get current positions of trackbars
    h_low = cv2.getTrackbarPos('H_low',name)
    s_low = cv2.getTrackbarPos('S_low',name)
    v_low = cv2.getTrackbarPos('V_low',name)

    # get current positions of trackbars
    h_hi = cv2.getTrackbarPos('H_high',name)
    s_hi = cv2.getTrackbarPos('S_high',name)
    v_hi = cv2.getTrackbarPos('V_high',name)

    return [h_low, s_low, v_low, h_hi, s_hi, v_hi]
########


# last resort
def BaxterMovement(p,q):
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rs.enable()

    limb = 'right'
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
    'left': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q)),
    'right': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q))}

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        limb = baxter_interface.Limb('right')
        limb.move_to_joint_positions(limb_joints)
        print "\nIK Joint Solution:\n", limb_joints

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return

def imagecb(data):
    x = data.x
    y = data.y

    print x,y

    xfac = 530.0
    yfac = -464.0
    xp = x/xfac+0.01
    yp = y/yfac
    print xp
    print yp

    baxterright = baxter_interface.Gripper('right')
    baxterright.calibrate()
    baxterright.open()

    q = Quaternion(x=-0.0177807149532, y=0.999274143222, z=-0.00996636325311, w=-0.0321823626761)
    #p0 = Point(x=0.678147548375,y=-0.2908471534,z=-0.0351726872428)
    p1 = Point(x=xp, y=yp, z=-0.1851726872428)
    p2 = Point(x=xp,y=-yp,z=-0.1851726872428)
    p3 = Point(x=xp,y=yp,z=-0.0351726872428)
    p4 = Point(x=xp+0.064958,y=yp-0.061788,z=-0.0351726872428)
    p5 = Point(x=xp+0.064958,y=yp-0.061788,z=-0.1851726872428)
    p6 = Point(x=xp+0.064958,y=yp-0.061788,z=-0.0351726872428)
    p7 = Point(x=xp,y=yp,z=-0.0351726872428)

    BaxterMovement(p1,q)
    BaxterMovement(p2,q)
    baxterright.close()
    BaxterMovement(p3,q)
    BaxterMovement(p4,q)
    BaxterMovement(p5,q)
    baxterright.open()
    BaxterMovement(p6,q)
    BaxterMovement(p7,q)
#attempt to detect ar tags
def main():
    rospy.init_node("pickandplace")
    open_cv_working = True
    ar_code_working = False
    if open_cv_working:
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        # new baxter default
        rospy.loginfo("Enabling robot... ")
        rs.enable()
        bridge = CvBridge()
        rightcam = bi.CameraController('right_hand_camera')
        bridge = CvBridge()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        baxterright = baxter_interface.Gripper('right')
        baxterright.calibrate()
        baxterright.open()
        q = Quaternion(x=-0.0177807149532, y=0.999274143222, z=-0.00996636325311, w=-0.0321823626761)
        p0 = Point(x=0.678147548375,y=-0.2908471534,z=-0.0351726872428)
        BaxterMovement(p0,q)
        # real stuff in this function
        WaitForImage(bridge)
    elif ar_code_working:
        limb = 'right'
        hover_distance = 0.3 # meters
        pnp = PickAndPlace(limb, hover_distance)
        # Starting Joint angles for right arm
        starting_joint_angles = {'right_s0': 0.104694188773, 'right_s1': -0.385029177759, 'right_w0': -1.2279516207, 'right_w1': 1.7859371323, 'right_w2': 0.0521553467881, 'right_e0': 1.43158757029, 'right_e1': 0.734009807003}
        # Move to the desired starting angles
        pnp.move_to_start(starting_joint_angles)
        #subscriber node for the ar stuff, call back takes in the markers and the pnp obj
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, callback_args=(pnp,))
        try:
            rospy.spin()
    else:
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
    
        rospy.loginfo("Enabling robot... ")
        rs.enable()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        baxterright = baxter_interface.Gripper('right')
        baxterright.calibrate()
        baxterright.open()
        q = Quaternion(x=-0.0177807149532, y=0.999274143222, z=-0.00996636325311, w=-0.0321823626761)
        p0 = Point(x=0.678147548375,y=-0.2908471534,z=-0.0351726872428)
        BaxterMovement(p0,q)
        print 'a'
        rospy.sleep(1.5)
        print 'b'
        rospy.Subscriber("opencv/center_of_object", Point, imagecb, queue_size=1)
        try:
            rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
