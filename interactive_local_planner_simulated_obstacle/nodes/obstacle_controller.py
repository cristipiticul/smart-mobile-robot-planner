#!/usr/bin/env python
import rospy
import json
from interactive_local_planner_simulated_obstacle.moving_obstacle_controller import MovingObstacleController
from interactive_local_planner_simulated_obstacle.obstacle_pose_publisher import ObstaclePosePublisher

def main():
    rospy.init_node('obstacle_controller')
    
    distance_threshold = rospy.get_param('~distance_threshold')
    robot_link = rospy.get_param('~robot_link')
    obstacle_link = rospy.get_param('~obstacle_link')
    joints_to_move_str = rospy.get_param('~joints_to_move')
    joints_to_move = json.loads(joints_to_move_str)

    robot_tf_frame = rospy.get_param('~robot_tf_frame')
    published_tf_frame = rospy.get_param('~published_tf_frame')
    
    rospy.loginfo('obstacle_controller: Robot link: %s' % (robot_link,))
    rospy.loginfo('obstacle_controller: Obstacle link: %s' % (obstacle_link,))
    rospy.loginfo('obstacle_controller: Robot TF frame: %s' % (robot_tf_frame,))
    rospy.loginfo('obstacle_controller: Obstacle (published) TF frame: %s' % (published_tf_frame,))
    rospy.loginfo('obstacle_controller: Distance threshold: %f' % (distance_threshold,))
    rospy.loginfo('obstacle_controller: Joints to move: %s' % (str(joints_to_move),))
    
    MovingObstacleController(robot_link, obstacle_link, distance_threshold, joints_to_move)
    ObstaclePosePublisher(robot_link, obstacle_link, robot_tf_frame, published_tf_frame)
    
    rospy.spin()
    
    return 0


if __name__ == '__main__':
    main()
