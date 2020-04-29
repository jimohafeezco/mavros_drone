#!/usr/bin/env python

import rospy
import numpy as np
# import rospylatex
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray
from mavros_msgs.srv import CommandBool, SetMode
from dynamic_reconfigure.server import Server
from dynamic_reconf.cfg import DynamicConfig
from mavros_msgs.msg import State, ExtendedState
from dynamic_reconfigure.msg import Config

class DroneControl():

    def __init__(self, kp=3, kd=0.5,goal_vel=2.0):

        self.Kp= kp
        self.Kd= kd
        self.goal_vel= goal_vel
        cur_pos_topic = '/mavros/local_position/pose'
        goal_topic = '/goal'
        cur_vel_topic = '/mavros/local_position/velocity_local'
        check_topic = '/mavros/setpoint_velocity/cmd_vel'
        rospy.Subscriber(goal_topic, PoseStamped, self.set_goal)
        rospy.Subscriber(cur_pos_topic,PoseStamped,self.set_cur_pose)
        rospy.Subscriber(cur_vel_topic,TwistStamped,self.set_cur_vel)
        rospy.Subscriber(check_topic, TwistStamped, self.check_vel)
        rospy.Subscriber("/listener/parameter_updates", Config, self.callback)

        # rospy.Subscriber('/obstacles', PoseArray, self.obstacle)

        rospy.Subscriber('/mavros/state', State, self.state_cb)

        # arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        # mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        # mode = arming_srv(value=True)

        # mode_srv(custom_mode="OFFBOARD")
        # srv = Server(DynamicConfig, dynamic)

        topic_pub = '/mavros/setpoint_velocity/cmd_vel'
        self.pub = rospy.Publisher(topic_pub, TwistStamped, queue_size=10)


    def callback(self, msg):
        self.gains = msg
        for i in range(len(self.gains.doubles)):
            if self.gains.doubles[i].name =="kp" :
                # print("kp found")
                self.Kp=self.gains.doubles[i].value
            if self.gains.doubles[i].name =="kd" :
                # print("kd found")
                self.Kd=self.gains.doubles[i].value

        return self.Kp, self.Kd

    def state_cb(self, msg):
        # mode_srv(custom_mode="OFFBOARD")
        self.state_mode= msg
        # print(self.state_mode.mode)
        # self.state=msg
        if(self.state_mode.mode=='OFFBOARD'):
            isReadyToFly = True
            print("flying in progress")     

    def set_goal(self,msg):
        # global target_pos 
        self.target_pos = msg
        #print(target_pos)
        
    def set_cur_pose(self, msg):
        # global cur_pos
        self.cur_pos = msg
        #print(cur_pos)
        
    def set_cur_vel(self,msg):
        # global cur_vel
        self.cur_vel = msg

    def check_vel(self,msg):
        #print(msg)
        pass

    def trans_q_to_e(self,obj):
        self.qx = obj.pose.orientation.x
        self.qy = obj.pose.orientation.y
        self.qz = obj.pose.orientation.z
        self.qw = obj.pose.orientation.w   
        
        self.rotateZa0 = 2.0*(self.qx*self.qy + self.qw*self.qz)
        self.rotateZa1 = self.qw*self.qw + self.qx*self.qx - self.qy*self.qy - self.qz*self.qz
        self.rotateZ = 0.0
        if self.rotateZa0 != 0.0 and self.rotateZa1 != 0.0:
            self.rotateZ = np.arctan2(self.rotateZa0, self.rotateZa1)
        return self.rotateZ

    def update_velocity(self,u):
        # print(self.Kp)
        msg= Config()
        self.Kp = self.callback(msg)[0]
        self.Kd = self.callback(msg)[1]

        u.twist.linear.x = self.Kp * (self.target_pos.pose.position.x - self.cur_pos.pose.position.x) + self.Kd * (
                    self.goal_vel - self.cur_vel.twist.linear.x) 
        u.twist.linear.y = self.Kp * (self.target_pos.pose.position.y - self.cur_pos.pose.position.y) + self.Kd * (
                    self.goal_vel - self.cur_vel.twist.linear.y) 
        u.twist.linear.z = self.Kp * (self.target_pos.pose.position.z - self.cur_pos.pose.position.z) + self.Kd * (
                    self.goal_vel - self.cur_vel.twist.linear.z)

        if u.twist.linear.x > 2:
            u.twist.linear.x = 2
        elif u.twist.linear.x < -2:
            u.twist.linear.x = -2
        if u.twist.linear.y > 2:
            u.twist.linear.y = 2
        elif u.twist.linear.y < -2:
            u.twist.linear.y = -2
        if u.twist.linear.z > 1.5:
            u.twist.linear.z = 1.5
        elif u.twist.linear.z < -1.5:
            u.twist.linear.z = -1.5
        g_z_rot = self.trans_q_to_e(self.target_pos)
        c_z_rot = self.trans_q_to_e(self.cur_pos)
        u.twist.angular.z = self.Kp * (g_z_rot - c_z_rot) + self.Kd * (0.0 - self.cur_vel.twist.angular.z)
        u.header.stamp = rospy.Time.now()

      
    def sub_and_pub(self):
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        rate = rospy.Rate(10) # 10hz
        self.target_pos = PoseStamped()
        self.cur_pos = PoseStamped()
        self.cur_vel = TwistStamped()
        self.current_state= State()
        u = TwistStamped()
        u.header.frame_id = "map"
        self.previous_state= self.current_state
        # while not self.current_state.connected:
        #     rate.sleep()

        last_request = rospy.get_rostime()

        print("arming in progress")
        print("offboarding in progress")
        self.arming_srv(True)

        while not rospy.is_shutdown():

            # now = rospy.get_rostime()

            # if self.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            self.mode_srv(custom_mode="OFFBOARD")
            #     last_request = now
            # else:
            #     if not self.current_state.armed and (now - last_request > rospy.Duration(5.)):
            #         last_request = now

            # check the status
            # if self.previous_state.armed != self.current_state.armed:
            #     rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
            # if self.previous_state.mode != self.current_state.mode:
            #     rospy.loginfo("Current mode: %s" % self.current_state.mode)
            # self.previous_state = self.current_state

            self.update_velocity(u)     
            # self.state_cb(self.state_mode)
            self.pub.publish(u)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    control= DroneControl()
    control.sub_and_pub()


