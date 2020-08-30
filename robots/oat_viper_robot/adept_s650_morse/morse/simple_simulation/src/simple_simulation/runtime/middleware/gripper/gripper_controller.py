import logging
import rospy
from std_srvs.srv import Trigger
from morse_helpers import adapters


logger = logging.getLogger("morse." + __name__)


def _stamp_to_secs(self, stamp):
    return stamp.secs + stamp.nsecs / 1e9


class GripperController(adapters.ROSController):

    def __init__(self, overlaid_object, namespace=None):
        super().__init__(overlaid_object)

        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    @adapters.ros_service(type=Trigger, name="close_gripper")
    def close(self):        
        name = self.overlaid_object.grab()
        ret = (name is None)
        return ret, name
    
    @adapters.ros_service(type=Trigger, name="open_gripper")
    def open(self):
        ret = self.overlaid_object.release()
        return ret, ""
