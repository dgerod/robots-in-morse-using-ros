import logging; logger = logging.getLogger("morse." + __name__)

import roslib, rospy
from morse.core import status
from morse.core.services import interruptible
from morse.core.exceptions import MorseServiceError
from morse_helpers import adapters
from morse.middleware.socket_datastream import MorseEncoder

import json
from morse_msgs.srv import FindObject, GetObjects
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class SmartCameraPublisher(adapters.ROSPublisherTF):
    """ 
    Publish the data of the semantic camera as JSON in a ROS String message.
    And send TF transform between '/map' and ``object.name``.
    """
    ros_class = String
    default_frame_id = '/map'

    def initialize(self):
        if not self.component_instance.relative:
            self.default_frame_id = self.kwargs.get("frame_id", SmartCameraPublisher.default_frame_id)
        super().initialize()

    def default(self, ci='unused'):
        for obj in self.data['visible_objects']:
            # send tf-frame for every object
            self.sendTransform(obj['position'], obj['orientation'],
                               self.get_time(), str(obj['name']), self.frame_id)
        string = String()
        string.data = json.dumps(self.data['visible_objects'], cls=MorseEncoder)
        self.publish(string)

class SmartCameraCtrlByServices(adapters.ROSController):

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

    @adapters.ros_service(type = GetObjects)
    def get_objects(self):        
        
        logger.info("[SmartCameraController::get_objects] Begin")

        names = []
        types = []
        poses = []
        
        for obj in self.overlaid_object.local_data['visible_objects']:
            
            logger.info("Object: " + str(obj['name']))
           
            names.append(String(obj['name']))
            types.append(String(obj['type']))
            
            pose = Pose()        
            pose.position = obj['position']
            pose.orientation = obj['orientation']
            poses.append(pose)
          
        logger.info("[SmartCameraController::get_objects] End")
        return (names, types, poses)
    
    # ------------------------------------------------------
        
    @adapters.ros_service(type = FindObject)
    def find_object(self, req):        
        
        logger.info("[SmartCameraController::find_object] Begin")

        success = False
        type = String()
        pose = Pose()

        name = req.data
        logger.info("Looking for object: " + name)

        for obj in self.overlaid_object.local_data['visible_objects']:

            logger.info("Object: " + str(obj['name']))
            if str(obj['name']) == name:
                pose.position = obj['position']
                pose.orientation = obj['orientation']  
                type = String(obj['type'])
                success = True
                break

        logger.info("Type: " + str(type))
        logger.info("[SmartCameraController::find_object] End")
        return (success, type, pose)
