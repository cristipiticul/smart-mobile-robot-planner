import rospy
import exceptions
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

class GazeboInterface:

    def __init__(self, robot_link, world_link):
        rospy.loginfo('gazebo_interface: Waiting for service "/gazebo/get_link_state"')
        rospy.wait_for_service('/gazebo/get_link_state')
        rospy.loginfo('gazebo_interface: Done')
        self._get_link_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self._robot_link = robot_link
        self._world_link = world_link

    def get_robot_pose(self):
        request = GetLinkStateRequest()
        request.link_name = self._robot_link
        request.reference_frame = self._world_link
        response = self._get_link_state_service.call(request)
        if not response.success:
            raise exceptions.RuntimeError('Problem when getting pose between %s and %s. Details: %s' % (self._world_link, self._robot_link, response.status_message))
        return response.link_state.pose
