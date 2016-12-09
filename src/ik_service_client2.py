#!/usr/bin/env python

import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION
import baxter_interface
from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import Point

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
    xp = x/xfac+0.08
    yp = y/yfac-0.04
    print xp
    print yp

    # xp = 0.678147548375
    # yp = -0.2908471534

    baxterright = baxter_interface.Gripper('right')
    baxterright.calibrate()
    baxterright.open()

    q = Quaternion(x=-0.0177807149532, y=0.999274143222, z=-0.00996636325311, w=-0.0321823626761)
    #p0 = Point(x=0.678147548375,y=-0.2908471534,z=-0.0351726872428)
    h = 2*0.0245
    p1 = Point(x=xp,y=yp,z=-0.0451726872428+2*h)
    p2 = Point(x=xp,y=yp,z=-0.1951726872428+2*h)
    p3 = Point(x=xp,y=yp,z=-0.0451726872428+2*h)
    p4 = Point(x=xp+0.064958,y=yp-0.051788,z=-0.0451726872428)
    p5 = Point(x=xp+0.064958,y=yp-0.051788,z=-0.1951726872428)
    p6 = Point(x=xp+0.064958,y=yp-0.051788,z=-0.0451726872428)
    #red
    p7 = Point(x=xp,y=yp,z=-0.0451726872428+h)
    p8 = Point(x=xp,y=yp,z=-0.1951726872428+h)
    p9 = Point(x=xp,y=yp,z=-0.0451726872428+h)
    p10 = Point(x=xp-0.084958,y=yp-0.051788,z=-0.0451726872428)
    p11 = Point(x=xp-0.084958,y=yp-0.051788,z=-0.1951726872428)
    p12 = Point(x=xp-0.084958,y=yp-0.051788,z=-0.0451726872428)
    #blue
    p13 = Point(x=xp,y=yp,z=-0.0451726872428)
    p14 = Point(x=xp,y=yp,z=-0.1951726872428)
    p15 = Point(x=xp,y=yp,z=-0.0451726872428)
    p16 = Point(x=xp-0.084958,y=yp+0.051788,z=-0.0451726872428)
    p17 = Point(x=xp-0.084958,y=yp+0.051788,z=-0.1951726872428)
    p18 = Point(x=xp-0.084958,y=yp+0.051788,z=-0.0451726872428)
    #green
    BaxterMovement(p1,q)
    BaxterMovement(p2,q)
    baxterright.close()
    BaxterMovement(p3,q)
    BaxterMovement(p4,q)
    BaxterMovement(p5,q)
    baxterright.open()
    BaxterMovement(p6,q)
    BaxterMovement(p7,q)
    BaxterMovement(p8,q)
    baxterright.close()
    BaxterMovement(p9,q)
    BaxterMovement(p10,q)
    BaxterMovement(p11,q)
    baxterright.open()
    BaxterMovement(p12,q)
    BaxterMovement(p13,q)
    BaxterMovement(p14,q)
    baxterright.close()
    BaxterMovement(p15,q)
    BaxterMovement(p16,q)
    BaxterMovement(p17,q)
    baxterright.open()
    BaxterMovement(p18,q)

def main():
    rospy.init_node("baxter_movement")

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

    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
