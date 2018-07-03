#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
import tf2_ros
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

TF_PUBLISHER_FREQUENCY = 10.0 # in Hz - how often to publish the obstacle position to TF
class ObstaclePosePublisher:
    def __init__(self, robot_link, obstacle_link, robot_tf_frame, published_tf_frame):
        self.robot_link = robot_link
        self.obstacle_link = obstacle_link
        self.robot_tf_frame = robot_tf_frame
        self.published_tf_frame = published_tf_frame
        self.get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Timer(rospy.Duration(1.0 / TF_PUBLISHER_FREQUENCY), self.publish_position)
    
    def publish_position(self, event):
        request = GetLinkStateRequest()
        request.link_name = self.obstacle_link
        request.reference_frame = self.robot_link
        response = self.get_link_state_service.call(request)
        if not response.success:
            rospy.logerr('obstacle_controller: Problem when getting relative pose between robot and obstacle')
            rospy.logerr('obstacle_controller: Details: %s', response.status_message)
            return
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.robot_tf_frame
        transform.child_frame_id = self.published_tf_frame
        transform.transform.translation.x = response.link_state.pose.position.x
        transform.transform.translation.y = response.link_state.pose.position.y
        transform.transform.translation.z = response.link_state.pose.position.z
        transform.transform.rotation = response.link_state.pose.orientation
        self.tf_broadcaster.sendTransform(transform)