#!/usr/bin/env python

import rospy,math, time
from nav_msgs.msg import Odometry
import numpy as np

class ExplorationMetrics():

    def __init__(self):
        self.total_travel = 0
        self.current_pose = None
        self.prev_pose = None
        self.rate = 5
        self.odom_received = False

        '''Subscribers'''
        rospy.Subscriber('uav1/odometry/odom_main', Odometry, self.globalPositionCallback, queue_size=1)

    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.odom_received:
                if np.linalg.norm(self.current_pose - self.prev_pose) <= 0.05:
                    pass
                else:
                    self.total_travel += np.linalg.norm(self.current_pose - self.prev_pose)
                    self.prev_pose = self.current_pose
                print "TSP: Travel distance-> ", 
                print(round(self.total_travel,2))

            rate.sleep()

    def globalPositionCallback(self,msg):
        if not self.odom_received:
            self.prev_pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
        self.current_pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
        self.odom_received = True
    
if __name__ == '__main__':
    rospy.init_node('Exploration_Metrics')
    exp_metrics = ExplorationMetrics()
    exp_metrics.run()