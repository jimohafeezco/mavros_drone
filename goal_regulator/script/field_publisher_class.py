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



# pose.pose.position
class PotentialField():
    def __init__(self):

        pose_topic = "/mavros/local_position/pose"
        rospy.Subscriber(pose_topic, PoseStamped, self.pose_sub)

        velocity_topic = "/mavros/local_position/velocity_local"
        rospy.Subscriber(velocity_topic, TwistStamped, self.velocity_sub)

        rospy.Subscriber('/obstacles', PoseArray, self.obstacles_sub)
        rospy.Subscriber("/listener/parameter_updates", Config, self.dynamic_configure) # dynamics reconfigure subscriber

        self.field_pub = rospy.Publisher('/field', TwistStamped, queue_size=10)
        self.pose = PoseStamped()
        self.velocity = TwistStamped()
        self.obstacles = PoseArray()

        self.config = Config() 
        self.field = TwistStamped()
    def pose_sub(self,msg):
        # global pose
        self.pose = msg.pose.position

    def velocity_sub(self,msg):
        # global velocity
        self.velocity = msg

    def obstacles_sub(self,msg):
        # global obstacles
        self.obstacles = msg
        # print(obstacles.poses)
        # return obstacles


    def dynamic_configure(self,msg): # dynamics reconfigure callback
        # global force, tolerance
        self.config = msg
        for i in range(len(self.config.doubles)):

            if self.config.doubles[i].name == 'force':
                self.force = self.config.doubles[i].value
            if self.config.doubles[i].name == 'tolerance':
                self.tolerance = self.config.doubles[i].value
            if self.config.doubles[i].name == 'c':
                self.c = self.config.doubles[i].value

    def get_distance_obstacle(self,length,diff_x, diff_y, distance):
        self.length = length
        self.diff_x = diff_x
        self.diff_y = diff_y

        for i in range(self.length):
            self.diff_x[i] = self.pose.x - self.obstacles.poses[i].position.x
            self.diff_y[i] = self.pose.y - self.obstacles.poses[i].position.y
            self.distance[i] = np.hypot(self.diff_x[i], self.diff_y[i])
        if self.distance == []:
            self.nearest = 0
            self.nearest_distance=1
        else:
            self.nearest =np.argmin(self.distance)

            self.nearest_distance= self.distance[self.nearest]
        print(self.nearest, self.nearest_distance)
        return self.nearest, self.nearest_distance

    def power_of_field(self,force, tolerance, distance, c):
        self.tolerance = tolerance
        self.force = force
        self.distance = distance
        self.c =c
        self.power=force*(self.c/self.distance - self.c/self.tolerance)**2
        return self.power

    def potential_field(self, force = np.random.randint(30), tolerance = np.random.randint(2)/10):
        rospy.init_node('potential_field', anonymous=True)
        rate = rospy.Rate(20.0) 
        self.tolerance = tolerance
        self.force = force
        while not rospy.is_shutdown():

            self.length = len(self.obstacles.poses)

    #initializing distance to obstacles
            self.diff_x = [0] * self.length
            self.diff_y = [0] * self.length
            self.distance = [0] * self.length

    # getting nearest obstacle to drone
            self.nearest,self.nearest_distance = self.get_distance_obstacle(self.length,self.diff_x, self.diff_y, self.distance)
            
            if self.nearest_distance < self.tolerance:
                self.dir_x = diff_x[self.nearest] / np.hypot(self.diff_x[self.nearest] ,self.diff_y[self.nearest])
                self.dir_y = diff_y[self.nearest] / np.hypot(self.diff_x[self.nearest],  self.diff_y[self.nearest])

                self.power = power_of_self.field(self.force, self.tolerance, self.distance[self.nearest])

                self.field.twist.linear.x = self.power * self.dir_x
                self.field.twist.linear.y = self.power * self.dir_y
                self.field.twist.linear.z = 0
            else:
                self.field.twist.linear.x = 0
                self.field.twist.linear.y = 0
                self.field.twist.linear.z = 0

            # set the angular part of the potential filed
            self.field.twist.angular.x = 0
            self.field.twist.angular.y = 0
            self.field.twist.angular.z = 0

            # update timestamp
            self.field.header.stamp = rospy.Time.now()
            
            # publish control
            self.field_pub.publish(self.field)

            rate.sleep()


if __name__ == '__main__':
    try:
        potential=PotentialField()
        potential.potential_field()
    except rospy.ROSInterruptException:
        pass