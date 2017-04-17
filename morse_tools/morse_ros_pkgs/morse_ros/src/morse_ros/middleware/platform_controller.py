import logging; logger = logging.getLogger("morse."+ __name__)

from morse_helpers import adapters
from morse.core.blenderapi import mathutils
from morse_msgs.srv import MoveToPose

class PlatformStatePublisher(adapters.ROSPublisher):
    ros_class = None

    def default(self, ci='unused'):
        pass

class PlatformCtrlByServices(adapters.ROSController):

    def __init__(self, overlaid_object, namespace = None):
        super().__init__(overlaid_object)

        pose = list(overlaid_object.local_data.keys())
        self.namespace = namespace
        name = adapters.morse_to_ros_namespace( self.name() )

    def _stamp_to_secs(self, stamp):
        return stamp.secs + stamp.nsecs / 1e9

    def name(self):
        if self.namespace:
            return self.namespace
        else:
            return super().name()

    # ------------------------------------------------------

    @adapters.ros_service(type = MoveToPose)
    def move_to_pose(self, req):

        logger.info("[PlatformCtrlByServices::move_to_pose] End")

        pose = req        
        translation = [pose.position.x, pose.position.y, pose.position.z]
        orientation = mathutils.Quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)).to_euler()     
        
        logger.info(orientation)
        #self.overlaid_object.translate(translation[0], translation[1], translation[2])
        #self.overlaid_object.orientation(orientation[0], orientation[1], orientation[2])

        self.overlaid_object.local_data['x'] = translation[0]
        self.overlaid_object.local_data['y'] = translation[1]
        self.overlaid_object.local_data['z'] = translation[2]
        self.overlaid_object.local_data['roll'] = orientation[0]
        self.overlaid_object.local_data['pitch'] = orientation[1]
        self.overlaid_object.local_data['yaw'] = orientation[2]

        logger.info("[PlatformCtrlByServices::move_to_pose] End")
        return True

    # ------------------------------------------------------
