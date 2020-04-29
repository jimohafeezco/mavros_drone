#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_reconf.cfg import DynamicConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {kp}, {kd},{force},{tolerance},{c}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("listener", anonymous = False)

    srv = Server(DynamicConfig, callback)
    rospy.spin()