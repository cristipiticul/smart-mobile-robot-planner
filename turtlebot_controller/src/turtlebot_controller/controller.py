#! /usr/bin/env python
import rospy
import actionlib
from turtlebot_controller.msg import TurtleBotMovementAction, TurtleBotMovementResult

class TurtleBotController:

    def __init__(self, name):
        self._action_name = name

        self._cell_size = rospy.get_param('~cell_size')

        self._result = TurtleBotMovementResult()
        self._action_server = actionlib.SimpleActionServer(self._action_name,
            TurtleBotMovementAction, execute_cb=self.callback, auto_start=False)
        self._action_server.start()
        print('TurtleBotController: Action server %s started' % name)


    def callback(self, goal):
        print('TurtleBotController: Got command "%s"' % goal.command)

        if goal.command == 'forward':
            self.move_forward()
        elif goal.command == 'rotate_clockwise':
            self.rotate_clockwise()
        elif goal.command == 'rotate_counterclockwise':
            self.rotate_counterclockwise()
        else:
            print('TurtleBotController: Unknown command "%s"' % goal.command)

        result = TurtleBotMovementResult()
        result.success = True
        self._action_server.set_succeeded(result)


    def move_forward(self):
        print('moving forward...')

    def rotate_clockwise(self):
        print('rotating clockwise...')

    def rotate_counterclockwise(self):
        print('rotating coutnerclockwise...')

def main():
    rospy.init_node('turtlebot_controller')
    controller = TurtleBotController('turtlebot_controller')
    rospy.spin()
