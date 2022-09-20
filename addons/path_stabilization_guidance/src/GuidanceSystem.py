#!/usr/bin/env python

"""
@author: André Potes
@email: andre.potes@gmail.com
@date: 19/04/2022
@licence: MIT
"""
import rospy
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from auv_msgs.msg import NavigationStatus
from medusa_msgs.msg import mPStabilizationRefs, mPStabilizationTarget
from geometry_msgs.msg import Point, Vector3

class PathStabilizationGuidanceNode:
    """
    A ROS node for the path stabilization algorithm guidance system. This controller makes the AUV position/attitude to the
     desired position (nearby the target) and attitude (such as to point to the mission position).
    Receives:
    - Data information of the mission and target position.
    Outputs:
    - Information formatted for the path stabilization algorithm regarding the mission and target position
        in the configured topics.
    """

    def __init__(self):
        """
        Class constructor. Initializes the ros node, loads the parameters from the ros parameter server, creates the guidance system
        publishers and initializes the timer that publishes the desired references for the outer-loop.
        """

        # ---Initialize the ROS NODE---
        rospy.init_node('path_stabilization_guidance')

        self.node_frequency = rospy.get_param('~node_frequency', 10)

        # --Load actor name to identify which index to search for in the gazebo message
        self.actor_name = rospy.get_param('~gazebo/actor_name', 'None')

        # ---Load guidance parameters and initialize references
        self.map_origin_utm = np.array(rospy.get_param('~guidance_params/map_origin_utm', [6299543.18, 164532.91]))
        self.using_mission = rospy.get_param('~guidance_params/using_mission', True)
        self.mission_pos = np.array(rospy.get_param('~guidance_params/mission_location', [0.0, 0.0, 0.0]))
        self.mission_pos[0] += self.map_origin_utm[0]
        self.mission_pos[1] += self.map_origin_utm[1]
        self.k_psi = rospy.get_param('~guidance_params/attitude/heading_ref/k_psi', 0.1)
        self.sigma_m = rospy.get_param('~guidance_params/attitude/heading_ref/sigma_m', 1.0)

        # ---Load the ROS configurations---
        self.initializePublishers()
        self.initializeSubscribers()

        # ---Start the ROS NODE callback---
        self.initializeTimer()

        # flag to get the first values when subscribing to the gazebo state update callback regarding velocity
        self.first_iteration_flag = True

        # flags to not compute and publish the references in case no vehicle state has been received or information from the target
        self.has_received_target_state = False
        self.has_received_vehicle_state = False


    def initializeTimer(self):
        """
        Method that starts the system timer that periodically calls a callback
        """
        self.timer = rospy.Timer(rospy.Duration(1/self.node_frequency), self.timerCallback)

    def initializePublishers(self):
        """
        Method that initializes the ROS publishers 
        """
        rospy.loginfo('Initializing Publishers for PathStabilizationGuidanceNode')

        # Defining the publishers for the Path Stabilization algorithm (outer loop)        
        self.mission_refs_pub = rospy.Publisher(rospy.get_param('~topics/publishers/references'), mPStabilizationRefs, queue_size=1)
        self.target_state_pub = rospy.Publisher(rospy.get_param('~topics/publishers/target_state'), mPStabilizationTarget, queue_size=1)

    def initializeSubscribers(self):
        """
        Method that initializes the ROS subscribers
        """
        rospy.loginfo('Initializing Subscribers for PathStabilizationGuidanceNode')
        
        # Subscribe to the topics possessing information of the vehicle
        rospy.Subscriber(rospy.get_param('~topics/subscribers/target_state'), ModelStates, self.updateTargetState)

        # Subscribe to the vehicle state
        rospy.Subscriber(rospy.get_param('~topics/subscribers/vehicle_state'), NavigationStatus, self.updateVehicleState)

    def timerCallback(self, event):
        """
        Callback used to publish to the outer loops the desired references
        :param event: A timer event - unused but required by the ROS API
        """
        if (not self.has_received_target_state) or (not self.has_received_vehicle_state):
            return

        # heading without side-slip (zeta = course angle - actual heading
        zeta1 = math.atan2(self.vehicle_body_vel[1] - self.vehicle_pos[1], self.vehicle_body_vel[0] - self.vehicle_pos[0] )

        #zeta1 = wrapTo2Pi(zeta1);
        # desired heading (with side-slip)
        delta1 = math.atan2( self.mission_pos[1] - self.vehicle_pos[1] , self.mission_pos[0] - self.mission_pos[0] )

        # angle control law 
        k1 = (1/2)*(math.tanh( (np.linalg.norm(self.vehicle_pos - self.target_pos) - self.sigma_m)/self.k_psi ) + 1 )

        roll_d = 0.0
        pitch_d = 0.0
        psi_d = k1*zeta1 + (1 - k1)*delta1
        
        # create the message to be published
        msg_refs = mPStabilizationRefs()
        msg_target = mPStabilizationTarget()

        msg_refs.header.stamp = rospy.Time.now()
        msg_target.header.stamp = rospy.Time.now()

        # Define the message references
        msg_target.target_pos = Point(self.target_pos[0], self.target_pos[1], self.target_pos[2])

        if (self.first_iteration_flag):
            # first iteration where we do not have access to previous data
            msg_target.target_vel = Vector3(0.0, 0.0, 0.0)

            self.first_iteration_flag = False
        else:
            dt = (rospy.Time.now() - self.t_last).to_sec()

            target_lin_vel_x = (msg_target.target_pos.x - self.target_pos_last.x ) / dt 
            target_lin_vel_y = (msg_target.target_pos.y - self.target_pos_last.y ) / dt 
            target_lin_vel_z = (msg_target.target_pos.z - self.target_pos_last.z ) / dt 

            msg_target.target_vel = Vector3(target_lin_vel_x, target_lin_vel_y, target_lin_vel_z)

        msg_refs.mission_pos = Point(self.mission_pos[0], self.mission_pos[1], self.mission_pos[2])
        msg_refs.desired_roll = roll_d
        msg_refs.desired_pitch = pitch_d
        msg_refs.desired_heading = psi_d

        self.mission_refs_pub.publish(msg_refs)
        self.target_state_pub.publish(msg_target)

        self.target_pos_last = msg_target.target_pos
        self.t_last = rospy.Time.now()
        

    def updateVehicleState(self, msg):
        """
        Receives state information from the vehicle navigation filter and updates current state
        """
        self.has_received_vehicle_state = True
        
        self.vehicle_body_vel = np.array([msg.body_velocity.x, msg.body_velocity.y, msg.body_velocity.z])
        self.vehicle_pos = np.array([msg.position.north, msg.position.east, msg.position.depth])


    def updateTargetState(self, msg):
        """
        Receives the model states message from gazebo and updates the target state (position and velocity).
        """
        if self.actor_name == 'None':
            return

        self.has_received_target_state = True


        actor_index = msg.name.index(self.actor_name)
        target_pos_enu = msg.pose[actor_index].position

        # position from actor in ENU but we work in NED convetion
        self.target_pos = np.array([target_pos_enu.y + self.map_origin_utm[0], target_pos_enu.x + self.map_origin_utm[1], -target_pos_enu.z])

        # when mission position is not defined, assume it to be equal to the target position
        if not self.using_mission:
            self.mission_pos = self.target_pos
            
def main():
    """
    Initialize the GuidanceSytemNode and let the timer callback do all the work
    """
    guidance_system_node = PathStabilizationGuidanceNode()
    rospy.spin()

if __name__ == '__main__':
    main()
