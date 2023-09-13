#!/usr/bin/env python 

import rospy,math, time
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint

from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse

from mrs_msgs.msg import TrajectoryReference, Reference

import copy


class BasicTrajectory: 

    def __init__(self):
        self.state = 'start'
        self.rate = 10
        self.current_pose = Pose()
        self.tarjet_pose = PoseStamped()
        self.current_reference = PoseArray()
        self.first_measurement_received = False

        # Publishers
        self.trajectory_pub = rospy.Publisher('/uav1/control_manager/trajectory_reference', TrajectoryReference, queue_size=1)

        # Subscribers
        rospy.Subscriber('uav1/odometry/odom_main', Odometry, self.globalPositionCallback, queue_size=1)
        rospy.Subscriber('uav1/control_manager/mpc_tracker/mpc_reference_debugging',PoseArray, self.referenceCallback, queue_size=1)


        #Init the service
        print("Waiting for service multi_dof_trajectory")
        rospy.wait_for_service('uav1/multi_dof_trajectory', timeout = 30)
        self.plan_trajectory_service = rospy.ServiceProxy(
            "uav1/multi_dof_trajectory", MultiDofTrajectory)

    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown() and not self.first_measurement_received:
            print("Waiting for the first pose...")
            time.sleep(1)
        print("Drone is at", self.current_pose.position)
        print("First pose received. Starting Navigation")

        while not rospy.is_shutdown():

            if self.state == 'start':
                request = MultiDofTrajectoryRequest()
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = [self.current_reference.position.x, \
                    self.current_reference.position.y, \
                    self.current_reference.position.z, \
                    self.quaternion2Yaw(self.current_reference.orientation)]
                request.waypoints.points.append(copy.deepcopy(trajectory_point))
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = [0,10,1,0]
                request.waypoints.points.append(copy.deepcopy(trajectory_point))
                request.publish_path = True
                request.publish_trajectory = True
                request.plan_path = True #Este en True grafica el path en RVIZ
                request.plan_trajectory = True # Este ejecuta la trajectoria y mueve el dron

                response = self.plan_trajectory_service(request)
                if response.success == False: 
                    print ("************")
                    print("Plan Has Failed")
                else: 
                    print("hasta", response.trajectory.points[-1].positions[0],response.trajectory.points[-1].positions[1],response.trajectory.points[-1].positions[2])
                    print('\n')
                    # if response.success == True:
                    # #     # self.trajectory_pub.publish(response.trajectory)
                    # #     self.state = 'Executing'

                    traj = TrajectoryReference()
                    print("tengo estos puntos en lista",len(response.trajectory.points))
                    


                    for i in range(len(response.trajectory.points)):
                        pt = Reference()
                        pt.position.x = response.trajectory.points[i].positions[0]
                        pt.position.y = response.trajectory.points[i].positions[1]
                        pt.position.z = response.trajectory.points[i].positions[2]
                        traj.points.append(pt)
                    self.state = 'Valid'

                # pt.position.x = response.trajectory.points[250].positions[0]
                # pt.position.y = response.trajectory.points[250].positions[1]
                # traj.points.append(pt)

                # pt.position.x = response.trajectory.points[-1].positions[0]
                # pt.position.y = response.trajectory.points[-1].positions[1]
                # traj.points.append(pt)

            if self.state == 'Valid':
                print("Flying there ----->")
                traj.fly_now = True
                traj.dt = 0.01
                self.trajectory_pub.publish(traj)
                self.state = "Executing"
                # print(response.trajectory.points[0]) #Asi es como se accede al primer punto de la trajectoria
                #quize esto se pueda luego adaptar a la forma en como el MPC recibe los puntos
            # if self.state == "Moving":

            rate.sleep()
    
    def referenceCallback(self,msg):
        self.current_reference = msg.poses[0]
        # print(self.current_reference)

    def globalPositionCallback(self,msg):
        self.current_pose = msg.pose.pose
        self.first_measurement_received = True
    
    def quaternion2Yaw(self, quaternion):
        q0 = quaternion.w
        q1 = quaternion.x
        q2 = quaternion.y
        q3 = quaternion.z
        return math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3))


if __name__ == '__main__':
    rospy.init_node('basic_trajectory')
    basic_trajectory = BasicTrajectory()
    basic_trajectory.run()
