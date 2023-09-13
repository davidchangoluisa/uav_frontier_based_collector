#!/usr/bin/env python 

import rospy,math, time
from enum import Enum
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Header, ColorRGBA, Float64, Empty, Int32, Bool
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint

from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse

from mrs_msgs.msg import TrajectoryReference, Reference

import copy

class ExplorationState(Enum):
    OFF = 0
    CHECKFORFRONTIERS = 1
    ON                = 2
    POINTREACHED      = 3
    AVOIDING          = 4

class FrontierExplorationSM: 

    def __init__(self):
        self.state = 'start'
        self.state_previous = 'none'
        self.rate = 10
        self.current_pose = Pose()
        self.current_traj = PoseStamped()
        self.prev_traj = PoseStamped()
        self.tarjet_pose = PoseStamped()
        self.current_reference = PoseArray()
        self.first_measurement_received = False
        self.executing_state = 0
        self.confirmed = False
        self.service_called = False

        '''Publishers'''
        self.trajectory_pub    = rospy.Publisher('/uav1/control_manager/trajectory_reference', TrajectoryReference, queue_size=1)
        self.path_pub          = rospy.Publisher('/uav1/path', Marker, queue_size=10)
        self.point_reached_pub = rospy.Publisher('/octomanager/exploration/point_reached',Bool,queue_size=1)

        '''Subscribers'''
        rospy.Subscriber('uav1/odometry/odom_main', Odometry, self.globalPositionCallback, queue_size=1)
        rospy.Subscriber('uav1/control_manager/mpc_tracker/mpc_reference_debugging',PoseArray, self.referenceCallback, queue_size=1)
        rospy.Subscriber('uav1/exploration/goal', PoseStamped, self.targetPointCallback,queue_size=1)
        rospy.Subscriber('octomanager/exploration/state', Int32, self.explorationStatusCallback, queue_size=1)


        #Init the service
        print("Waiting for service multi_dof_trajectory")
        rospy.wait_for_service('uav1/multi_dof_trajectory', timeout = 30)
        self.plan_trajectory_service = rospy.ServiceProxy(
            "uav1/multi_dof_trajectory", MultiDofTrajectory)

        rospy.Service('/uav1/confirm_trajectory', SetBool, self.confirm_trajectory)

    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown() and not self.first_measurement_received:
            print("Waiting for the first pose...")
            time.sleep(1)
        print("Drone is at", self.current_pose.position)
        print("First pose received. Starting Navigation")

        while not rospy.is_shutdown():

            if self.state == 'start':
                if self.state_previous !=self.state:
                    self.printStates()
                    self.state_previous = "start"

            if self.state == 'plan':
                if self.state_previous != self.state:
                    self.printStates()
                    self.state_previous = "plan"
                else:
                    print('Planning again')

                print('Calling Service')
                request = MultiDofTrajectoryRequest()
                trajectory_point = JointTrajectoryPoint()
                # Parsing starting point from MPC Tracker
                trajectory_point.positions = [self.current_reference.position.x, \
                    self.current_reference.position.y, \
                    self.current_reference.position.z, \
                    self.quaternion2Yaw(self.current_reference.orientation)]
                request.waypoints.points.append(copy.deepcopy(trajectory_point))
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = [self.target_pose.position.x, \
                    self.target_pose.position.y, self.target_pose.position.z, \
                    self.quaternion2Yaw(self.target_pose.orientation)]
                
                request.waypoints.points.append(copy.deepcopy(trajectory_point))
                request.plan_path = True 
                request.plan_trajectory = True 

                response = self.plan_trajectory_service(request)
                # if response.success == False:
                #     print("Plan Has Failed")
                #     self.state = ('end')
                #     self.point_reached_pub.publish(True)

                # else:
                self.publish_path(response.trajectory.points) # Just publish the path but do not execute it yet

                # while not self.service_called and not rospy.is_shutdown(): # Waiting until the confimation has been received
                #     rospy.sleep(0.01)

                # self.service_called = False  
                self.confirmed = True

                if self.confirmed:  
                    if response.success == False: 
                        print("Plan Has Failed")
                        self.returnToSafe()
                        self.state = ('end')
                        self.point_reached_pub.publish(True)
                        
                    else: 
                        self.current_traj = TrajectoryReference()
                        for i in range(len(response.trajectory.points)):
                            pt = Reference()
                            pt.position.x = response.trajectory.points[i].positions[0]
                            pt.position.y = response.trajectory.points[i].positions[1]
                            pt.position.z = response.trajectory.points[i].positions[2]
                            self.current_traj.points.append(pt)
                            self.current_traj.fly_now = True
                            self.current_traj.dt = 0.01
                        self.trajectory_pub.publish(self.current_traj)
                        self.state = "execute"
                        self.prev_traj = self.current_traj
                        print("Flying there ----->")
                else:
                    self.state = "end"  

            if self.state == "execute":
                if self.state_previous != self.state:
                    self.printStates()
                    self.state_previous = "execute"
                    self.execution_start = time.time()
                
                while not rospy.is_shutdown():
                    if self.checkTrajectoryExecuted() == True:
                        print("***Trajectory Finished***")
                        self.state = "end"
                        break
                    if self.state == "plan":
                        # self.stopTrajectory()
                        self.state == "end"
                        print('Trajectory Interrupted')
                        self.point_reached_pub.publish(True)

                        break
                    rate.sleep()

            if self.state == "end":
                if self.state_previous != self.state:
                    self.printStates()
                    self.state_previous = "end"
                    self.point_reached_pub.publish(True)
            
                time.sleep(0.05)
                self.state = 'start'
                
            
            rate.sleep()
    
    def referenceCallback(self,msg):
        self.current_reference = msg.poses[0]

    def globalPositionCallback(self,msg):
        self.current_pose = msg.pose.pose
        self.first_measurement_received = True
    
    def targetPointCallback(self,msg):
        self.target_pose = msg.pose
        self.state = 'plan'
        print("New goal accepted!->")

    def confirm_trajectory(self,req):
        self.service_called = True
        self.confirmed = req.data
        if (req.data):
            print("Argument OK")
        else:
            print("Argument NOT OK")
        return SetBoolResponse(True,"Service confirm_trajectory called")
    
    def explorationStatusCallback(self, msg):

        self.exploration_state = msg.data
        if (self.exploration_state  == ExplorationState.AVOIDING.value and self.state_previous == 'execute') :
            self.state = 'plan'
            self.stopTrajectory()
    

    def checkTrajectoryExecuted(self):
        x_curr = self.current_reference.position.x
        y_curr = self.current_reference.position.y
        z_curr = self.current_reference.position.z

        x_ref  = self.target_pose.position.x
        y_ref  = self.target_pose.position.y
        z_ref  = self.target_pose.position.z

        dx = x_curr - x_ref
        dy = y_curr - y_ref
        dz = z_curr - z_ref

        delta = math.sqrt(dx*dx + dy*dy + dz*dz)

        if delta < 0.3:
            print("Waypoint reached")
            return True
        return False
    
    def stopTrajectory(self):
        traj = TrajectoryReference()
        pt = Reference()
        pt.position.x = self.current_reference.position.x
        pt.position.y = self.current_reference.position.y
        pt.position.z = self.current_reference.position.z
        traj.points.append(pt)
        traj.fly_now = True
        traj.dt = 0.01
        self.trajectory_pub.publish(traj)
        # self.state = 'Execute'
        print('Returning to save position')
        time.sleep(0.05)
    
    def returnToSafe(self):
        print('RESPALDO ES: ',len(self.prev_traj.points))
        traj = TrajectoryReference()
        pt = Reference()
        pt.position.x = self.prev_traj.points[-500].position.x
        pt.position.y = self.prev_traj.points[-500].position.y
        pt.position.z = self.prev_traj.points[-500].position.z
        traj.points.append(pt)
        traj.fly_now = True
        traj.dt = 0.01
        self.trajectory_pub.publish(traj)
        time.sleep(0.2)
        # self.state = 'Execute'
        # self.point_reached_pub.publish(True)
        print('-500 Points reached')



    
    def quaternion2Yaw(self, quaternion):
        q0 = quaternion.w
        q1 = quaternion.x
        q2 = quaternion.y
        q3 = quaternion.z
        return math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3))
    
    def publish_path(self,path):
        if len(path) > 1:
            print('Path has been published')
            m = Marker()
            m.header.frame_id = 'uav1/gps_origin'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.CUBE_LIST
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            m.action = Marker.ADD
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.05
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 0.15
            color_red.g = 0.88
            color_red.b = 0.75
            color_red.a = 1
      
            for i in range(len(path)):
                p = Point()
                p.x = path[i].positions[0]
                p.y = path[i].positions[1]
                p.z = path[i].positions[2]
                m.points.append(p)
                m.colors.append(color_red)
            
            self.path_pub.publish(m)

    def printStates(self):
        print ("----------------------------------------------------")
        print ("State changed. Previous state:", self.state_previous)
        print ("State changed. Current state:", self.state)
        print ("----------------------------------------------------")
        print (" ")


if __name__ == '__main__':
    rospy.init_node('frontier_exploration')
    frontier_exploration = FrontierExplorationSM()
    frontier_exploration.run()
