#! /usr/bin/env python
import math
import geometry_msgs.msg

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
