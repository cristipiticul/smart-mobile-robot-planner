#!/usr/bin/env python
import rospy
import math
from turtlebot_gazebo_room.gazebo_interface import GazeboInterface
from turtlebot_gazebo_room.util import angleToQuaternion
from geometry_msgs.msg import Point, Quaternion

class MovingObstacleController:
    def __init__(self, model_name, initial_x, initial_y, initial_z, direction, min_y, max_y, velocity, cell_size):
        self._gazebo_interface = GazeboInterface(model_name, 'ground_plane::link')

        self._x = initial_x
        self._y = initial_y
        self._z = initial_z
        self._direction = direction
        self._min_y = min_y
        self._max_y = max_y
        self._velocity = velocity
        self._cell_size = cell_size

        self._orientations = {
            'south': angleToQuaternion(math.pi / 2),
            'north': angleToQuaternion(-math.pi / 2)
        }

        self._timer = rospy.Timer(rospy.Duration(1.0 / velocity), self.move)

    # Move 1 cell forward, or turn if at min_y/max_y
    def move(self, event):
        if self._direction == 'south':
            if self._y >= self._min_y:
                self._y -= self._cell_size
            else: # turn around
                self._direction = 'north'
        else:
            if self._y <= self._max_y:
                self._y += self._cell_size
            else: # turn around
                self._direction = 'south'
        return self.update_gazebo_pose()

    def update_gazebo_pose(self):
        return self._gazebo_interface.set_model_pose(Point(self._x, self._y, self._z), self._orientations[self._direction])
