#!/usr/bin/env python

import rospy
from rotors_control import import *

if __name__ == '__main__':
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "firefly"
    req.link_name_2 = "firefly/left_end_effector"
    attach_srv.call(req)
