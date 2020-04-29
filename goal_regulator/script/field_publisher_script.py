#!/usr/bin/python

import math
import rospy
import mavros
import numpy as np
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray
from mavros_msgs.srv import CommandBool, SetMode
from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig
from mavros_msgs.msg import State, ExtendedState

from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from dynamic_reconf.cfg import DynamicConfig
from mavros_msgs.msg import State, ExtendedState
from dynamic_reconfigure.msg import Config

mavros.set_namespace()

pose = PoseStamped()
velocity = TwistStamped()
obstacles = PoseArray()

config = Config() 
field = TwistStamped()

# pose.pose.position
def pose_sub(msg):
    global pose
    pose = msg.pose.position

def velocity_sub(msg):
    global velocity
    velocity = msg

def obstacles_sub(msg):
    global obstacles
    obstacles = msg
    # print(obstacles.poses)
    # return obstacles


def dynamic_configure(msg): # dynamics reconfigure callback
    global force, tolerance
    config = msg
    for i in range(len(config.doubles)):

        if config.doubles[i].name == 'force':
            force = config.doubles[i].value
        if config.doubles[i].name == 'tolerance':
            tolerance = config.doubles[i].value
        if config.doubles[i].name == 'c':
            c = config.doubles[i].value

pose_topic = "/mavros/local_position/pose"
rospy.Subscriber(pose_topic, PoseStamped, pose_sub)

velocity_topic = "/mavros/local_position/velocity_local"
rospy.Subscriber(velocity_topic, TwistStamped, velocity_sub)

rospy.Subscriber('/obstacles', PoseArray, obstacles_sub)
rospy.Subscriber("/listener/parameter_updates", Config, dynamic_configure) # dynamics reconfigure subscriber

field_pub = rospy.Publisher('/field', TwistStamped, queue_size=10)

def get_distance_obstacle(length,diff_x, diff_y, distance):

    for i in range(length):
        diff_x[i] = pose.x - obstacles.poses[i].position.x
        diff_y[i] = pose.y - obstacles.poses[i].position.y
        distance[i] = np.hypot(diff_x[i], diff_y[i])
    if distance == []:
        nearest = 0
        nearest_distance=1
    else:
        nearest =np.argmin(distance)

        nearest_distance= distance[closest]

    return nearest, nearest_distance

def power_of_field(force, tolerance, distance, c):
    power=force*(c/distance - c/tolerance)**2
    return power

def potential_field(force = np.random.randint(30), tolerance = np.random.randint(2)/10):
    rospy.init_node('potential_field', anonymous=True)
    rate = rospy.Rate(20.0) 

    while not rospy.is_shutdown():

        length = len(obstacles.poses)

#initializing distance to obstacles
        diff_x = [0] * length
        diff_y = [0] * length
        distance = [0] * length

# getting closest obstacle to drone
        closest,closest_distance = get_distance_obstacle(length,diff_x, diff_y, distance)
        
        if closest_distance < tolerance:
            dir_x = diff_x[closest] / np.hypot(diff_x[closest] ,diff_y[closest])
            dir_y = diff_y[closest] / np.hypot(diff_x[closest],  diff_y[closest])

            power = power_of_field(force, tolerance, distance[closest])

            field.twist.linear.x = power * dir_x
            field.twist.linear.y = power * dir_y
            field.twist.linear.z = 0
        else:
            field.twist.linear.x = 0
            field.twist.linear.y = 0
            field.twist.linear.z = 0

        # set the angular part of the potential filed
        field.twist.angular.x = 0
        field.twist.angular.y = 0
        field.twist.angular.z = 0

        # update timestamp
        field.header.stamp = rospy.Time.now()
        
        # publish control
        field_pub.publish(field)

        rate.sleep()


if __name__ == '__main__':
    try:
        potential_field()
    except rospy.ROSInterruptException:
        pass