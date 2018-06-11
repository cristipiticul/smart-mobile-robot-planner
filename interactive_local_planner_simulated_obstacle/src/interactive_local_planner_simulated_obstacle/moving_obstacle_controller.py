#!/usr/bin/env python
import rospy
import math
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, ApplyJointEffort, ApplyJointEffortRequest

MOVING_DURATION = rospy.Duration(3.0)
FREQUENCY = 5.0 # in Hz - how often to check the distance between robot and obstacle

def vector_length(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

class MovingObstacleController:
    def __init__(self, robot_link, obstacle_link, distance_threshold, joints_to_move):
        self.robot_link = robot_link
        self.obstacle_link = obstacle_link
        self.distance_threshold = distance_threshold
        self.joints_to_move = joints_to_move
        self.moving = False
        self.get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.apply_joint_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Timer(rospy.Duration(1.0 / FREQUENCY), self.move_if_robot_is_close)

    def move_if_robot_is_close(self, event):
        request = GetLinkStateRequest()
        request.link_name = self.obstacle_link
        request.reference_frame = self.robot_link
        response = self.get_link_state_service.call(request)
        if not response.success:
            rospy.logerr('obstacle_controller: Problem when getting relative pose between robot and obstacle')
            rospy.logerr('obstacle_controller: Details: %s', response.status_message)
            return

        relative_position = response.link_state.pose.position
        distance = vector_length(relative_position)
        if not self.moving and distance <= self.distance_threshold:
            self.moving = True
            request = ApplyJointEffortRequest()
            for joint in self.joints_to_move:
                request.effort = 1.0
                request.joint_name = joint
                request.duration = MOVING_DURATION
                request.start_time = rospy.Time(0) # start now
                response = self.apply_joint_effort_service(request)
                if not response.success:
                    rospy.logerr('obstacle_controller: Problem when moving joint %s', joint)
                    rospy.logerr('obstacle_controller: Details: %s', response.status_message)
                    return
            rospy.sleep(MOVING_DURATION)
            self.moving = False