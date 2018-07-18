import rospy
import exceptions
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, ApplyJointEffort, ApplyJointEffortRequest, SetModelState, SetModelStateRequest

class GazeboInterface:

    def __init__(self, model_name, world_link):
        rospy.loginfo('gazebo_interface: Waiting for Gazebo services')
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo('gazebo_interface: Done')
        self._model_name = model_name
        self._world_link = world_link

        self._get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #self._apply_joint_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self._set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


    def get_robot_pose(self):
        request = GetModelStateRequest()
        request.model_name = self._model_name
        request.relative_entity_name = self._world_link
        response = self._get_model_state_service.call(request)
        if not response.success:
            rospy.logerr('Problem when getting pose between %s and %s. Details: %s' % (self._world_link, self._robot_link, response.status_message))
            return None
        return response.pose

    '''
    def apply_joint_effort(self, joint, effort, duration):
        request = ApplyJointEffortRequest()
        request.effort = effort
        request.joint_name = joint
        request.duration = duration
        request.start_time = rospy.Time(0) # start now
        response = self.apply_joint_effort_service(request)
        if not response.success:
            rospy.logerr('Problem when moving joint %s. Details: %s' % (joint, response.status_message))
            return False
        return True
    '''

    def set_model_pose(self, position, orientation):
        request = SetModelStateRequest()
        request.model_state.model_name = self._model_name
        request.model_state.pose.position = position
        request.model_state.pose.orientation = orientation
        request.model_state.reference_frame = '' # self._world_link makes it relative
        response = self._set_model_state_service(request)
        print(str(request))
        if not response.success:
            rospy.logerr('Problem when changing pose of model %s. Details: %s' % (model_name, response.status_message))
            return False
        return True
