#!/usr/bin/env python
import rospy
from interactive_local_planner_simulated_obstacle.obstacle_pose_publisher import ObstaclePosePublisher

def main():
    rospy.init_node('static_obstacle_tf_publisher')
    
    robot_link = rospy.get_param('~robot_link')
    obstacle_link = rospy.get_param('~obstacle_link')
    robot_tf_frame = rospy.get_param('~robot_tf_frame')
    published_tf_frame = rospy.get_param('~published_tf_frame')
    
    rospy.loginfo('static_obstacle_tf_publisher: Robot link: %s' % (robot_link,))
    rospy.loginfo('static_obstacle_tf_publisher: Obstacle link: %s' % (obstacle_link,))
    rospy.loginfo('static_obstacle_tf_publisher: Robot TF frame: %s' % (robot_tf_frame,))
    rospy.loginfo('static_obstacle_tf_publisher: Obstacle (published) TF frame: %s' % (published_tf_frame,))
    
    ObstaclePosePublisher(robot_link, obstacle_link, robot_tf_frame, published_tf_frame)
    
    rospy.spin()
    
    return 0


if __name__ == '__main__':
    main()
