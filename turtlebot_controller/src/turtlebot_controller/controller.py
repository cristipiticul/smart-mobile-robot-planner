#! /usr/bin/env python
import math
import rospy
import actionlib
import exceptions
import geometry_msgs.msg
from turtlebot_controller.msg import TurtleBotMovementAction, TurtleBotMovementResult
from turtlebot_controller.gazebo_interface import GazeboInterface

def dist2D(p1, p2):
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

def orientationDifference(o1, o2):
    return abs(o1.z - o2.z) + abs(o1.w - o2.w)

def angleToQuaternion(angle):
    q = geometry_msgs.msg.Quaternion()
    q.x = q.y = 0.0
    q.z = math.sin(angle / 2.0)
    q.w = math.cos(angle / 2.0)
    return q

class TurtleBotController:
    ROBOT_LINEAR_VELOCITY = 0.3
    ROBOT_ANGULAR_VELOCITY = 0.8
    ANGLES = [i * math.pi / 2 for i in range(4)]
    ORIENTATION_EPS = 0.03 # maximum allowed orientationDifference

    def __init__(self, name):
        rospy.set_param('/turtlebot_controller_action_server_ready', False)
        self._action_name = name

        self._small_movement_sleep_rate = rospy.Rate(10)
        self._cell_size = rospy.get_param('~cell_size')

        self._orientation_index = 0

        self._position_interface = GazeboInterface(
            'mobile_base::base_footprint',
            'ground_plane::link'
        )

        self._command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi',
            geometry_msgs.msg.Twist, queue_size=10)

        self._action_server = actionlib.SimpleActionServer(
            self._action_name,
            TurtleBotMovementAction,
            execute_cb = self.callback,
            auto_start = False #recommended
        )
        self._action_server.start()

        rospy.set_param('/turtlebot_controller_action_server_ready', True)
        rospy.loginfo('TurtleBotController: Action server %s started' % name)


    def callback(self, goal):
        rospy.loginfo('TurtleBotController: Got command "%s"' % goal.command)
        success = True
        try:
            if goal.command == 'forward':
                self.move_forward()
            elif goal.command == 'rotate_clockwise':
                self.rotate_clockwise()
            elif goal.command == 'rotate_counterclockwise':
                self.rotate_counterclockwise()
            elif goal.command == 'wait':
                self.wait()
            else:
                raise exceptions.RuntimeError('Unknown command "%s"' % goal.command)
        except exceptions.RuntimeError as ex:
            rospy.logwarn('TurtleBotController: Command %s failed. Reason: %s' % (goal.command, str(ex)))
            success = false

        result = TurtleBotMovementResult()
        result.success = success
        rospy.loginfo('TurtleBotController: Done!')
        self._action_server.set_succeeded(result)

    def move_forward(self):
        rospy.logdebug('moving forward...')
        command = geometry_msgs.msg.Twist()
        command.linear.x = TurtleBotController.ROBOT_LINEAR_VELOCITY
        command.angular.z = 0.0

        start_position = self._position_interface.get_robot_pose().position
        current_position = start_position
        while dist2D(current_position, start_position) \
                < self._cell_size:
            self._command_publisher.publish(command)
            self._small_movement_sleep_rate.sleep()
            current_position = self._position_interface.get_robot_pose().position
        self.stop()

    def stop(self):
        command = geometry_msgs.msg.Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0
        self._command_publisher.publish(command)

    def rotate_clockwise(self):
        rospy.loginfo('rotating clockwise...')
        next_orientation_index = (self._orientation_index - 1) % len(TurtleBotController.ANGLES)
        end_orientation = angleToQuaternion(TurtleBotController.ANGLES[next_orientation_index])
        self.rotate_to(end_orientation, -1)
        self._orientation_index = next_orientation_index

    def rotate_counterclockwise(self):
        rospy.loginfo('rotating coutnerclockwise...')
        next_orientation_index = (self._orientation_index + 1) % len(TurtleBotController.ANGLES)
        end_orientation = angleToQuaternion(TurtleBotController.ANGLES[next_orientation_index])
        self.rotate_to(end_orientation, +1)
        self._orientation_index = next_orientation_index

    def rotate_to(self, end_orientation, velocity_sign):
        command = geometry_msgs.msg.Twist()
        command.linear.x = 0.0
        command.angular.z = velocity_sign*TurtleBotController.ROBOT_ANGULAR_VELOCITY

        start_orientation = self._position_interface.get_robot_pose().orientation
        current_known_orientation = angleToQuaternion(TurtleBotController.ANGLES[self._orientation_index])
        if orientationDifference(start_orientation, current_known_orientation) > TurtleBotController.ORIENTATION_EPS:
            rospy.logwarn("TurtleBotController: the known orientation may be wrong.")

        current_orientation = start_orientation
        while orientationDifference(current_orientation, end_orientation) \
                > TurtleBotController.ORIENTATION_EPS:
            self._command_publisher.publish(command)
            self._small_movement_sleep_rate.sleep()
            current_orientation = self._position_interface.get_robot_pose().orientation
        self.stop()

    def wait(self):
        rospy.loginfo('waiting...')

def main():
    rospy.init_node('turtlebot_controller')
    controller = TurtleBotController('turtlebot_controller')
    rospy.spin()
