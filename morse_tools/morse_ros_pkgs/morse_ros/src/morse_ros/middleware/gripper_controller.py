import logging; logger = logging.getLogger("morse."+ __name__)

from morse.core.services import interruptible
from morse.core import status
from morse.core.exceptions import MorseServiceError
from morse_helpers import adapters

from std_srvs.srv import Trigger
from morse_comms.msg import OpenGripperAction, OpenGripperResult, OpenGripperFeedback
from morse_comms.msg import CloseGripperAction, CloseGripperResult, CloseGripperFeedback

class GripperCtrlByServices(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        super().__init__(overlaid_object)
        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())

    @staticmethod
    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # Services
    # ------------------------------------------------------
    
    @adapters.ros_service(type=Trigger)
    def close(self):        
        name = self.overlaid_object.grab()
        ret = (name is None)
        return ret, name
    
    @adapters.ros_service(type=Trigger)
    def open(self):
        ret = self.overlaid_object.release()
        return ret, ""


class GripperCtrlByActions(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        super().__init__(overlaid_object)
        self.namespace = namespace
        name = adapters.morse_to_ros_namespace(self.name())

    @staticmethod
    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # Actions
    # ------------------------------------------------------
    
    @interruptible
    @adapters.ros_action(type = OpenGripperAction)
    def close(self, req):
        self.overlaid_object.trajectory(
                self.chain_callback(self.close_result), OpenGripperResult)

    def close_result(self, result):
        return result

    @interruptible
    @adapters.ros_action(type=CloseGripperAction)
    def open(self, req):
        self.overlaid_object.trajectory(
            self.chain_callback(self.open_result), CloseGripperResult)

    def open_result(self, result):
        return result

    # ------------------------------------------------------
