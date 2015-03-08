#!/usr/bin/env python  
import rospy
 
import math
import tf2_ros
import geometry_msgs.msg
import time

import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION




def set_j(limb, joint_name, angle):
    joint_command = {joint_name: angle}    
    limb.move_to_joint_positions(joint_command, timeout=0.3)

 
if __name__ == '__main__':
    rospy.init_node('kinect_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
 
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    lj = left.joint_names()
    rj = right.joint_names()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('left_shoulder_2', 'left_elbow_2', rospy.Time())

            print trans

            joint_position_xy = math.atan2(trans.transform.translation.x, trans.transform.translation.y)
            joint_position_yz = math.atan2(trans.transform.translation.y, trans.transform.translation.z)

            print joint_position_xy
            print joint_position_yz


            set_j(left, lj[0], joint_position_xy)
            set_j(left, lj[1], joint_position_xy)       

            
            time.sleep(0.1)



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:

            print e
            time.sleep(0.1)
            continue
 

