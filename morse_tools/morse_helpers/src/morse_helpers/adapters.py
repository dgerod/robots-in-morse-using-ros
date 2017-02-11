# =============================================================================
# Adapters to use MORSE with ROS.
# dgerod@xyz-lab.org.es
# =============================================================================

from morse_helpers import morse_local_config as settings
from morse.middleware.ros import ROSPublisher, ROSPublisherTF, ROSSubscriber
from morse.middleware.ros_request_manager import ros_service, ros_action
from morse.core.overlay import MorseOverlay

from morse.builder import Component
from morse.core.exceptions import MorseServiceError

# -----------------------------------------------------------------------------

# Classes already existing in MORSE for publishers/suscribers
#   ROSPublisher 
#   ROSSubscriber
#   ROSPublisherTF
# And our class to crete actions and services
#   ROSController
#
# All the components are register using ROSRegister class.

class ROSController(MorseOverlay):
    pass

class ROSRegister:
    _mw_location = settings.mw_loc;     
    
    @staticmethod
    def add_topic(component, name, topic_class=None, **kwargs):
        if topic_class is not None:
            path_info = ROSRegister._mw_location + topic_class
            component.add_stream("ros", path_info, topic = name, **kwargs)
        else:
            component.add_stream("ros", topic = name, **kwargs)
        
    @staticmethod
    def add_controller(component, name, controller_class):
        path_info = ROSRegister._mw_location + controller_class
        component.add_overlay("ros", path_info, namespace = name)
        
    @staticmethod
    def add(component, name, custom_class=None):
        pass
 
def morse_to_ros_namespace(name):
    return name.replace(".", "/")
  
# =============================================================================

