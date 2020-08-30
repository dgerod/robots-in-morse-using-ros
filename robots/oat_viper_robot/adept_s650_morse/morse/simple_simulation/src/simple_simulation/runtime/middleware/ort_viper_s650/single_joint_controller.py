import logging
import rospy
from morse_comms.srv import SetTargetJoints
from morse_helpers import adapters


logger = logging.getLogger("morse." + __name__)


def _stamp_to_secs(self, stamp):
    return stamp.secs + stamp.nsecs / 1e9


class SingleJointController(adapters.ROSController):

    def __init__(self, overlaid_object, namespace=None):

        super().__init__(overlaid_object)
        joints = list(overlaid_object.local_data.keys())

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace( self.name())
        rospy.set_param(name + "/joint_names", joints)

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    @adapters.ros_service(type=SetTargetJoints, name="set_target_joints")
    def _set_target_joints(self, request):

        joints = request.positions
        if len(joints) == 6:
            self.overlaid_object.set_target_joints(joints)
            return True

        return False
