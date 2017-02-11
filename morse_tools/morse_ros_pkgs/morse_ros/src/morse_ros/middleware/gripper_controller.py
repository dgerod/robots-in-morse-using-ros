import logging; logger = logging.getLogger("morse."+ __name__)

import roslib, rospy
from morse.core.services import interruptible
from morse.core import status
from morse.core.exceptions import MorseServiceError
from morse_helpers import adapters

from std_srvs.srv import Trigger
from anchoring_msgs.msg import DropObjectAction, DropObjectResult, DropObjectFeedback

class GripperCtrlByServices(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        super().__init__(overlaid_object)
        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # ------------------------------------------------------
    
    @adapters.ros_service(type = Trigger)
    def close(self):        
        name = self.overlaid_object.grab()
        ret = (name != None)
        return (ret, name)
    
    @adapters.ros_service(type = Trigger)
    def open(self):
        ret = self.overlaid_object.release()
        return (ret, "")


class GripperCtrlByActions(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        super().__init__(overlaid_object)
        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # ------------------------------------------------------
    
    @interruptible
    @adapters.ros_action(type = DropObjectAction)
    def drop_object(self, req):
        self.overlaid_object.trajectory(
                self.chain_callback(self.drop_object_result), DropObjectResult)

    def drop_object_result(self, result):
        return result
    
    # ------------------------------------------------------
