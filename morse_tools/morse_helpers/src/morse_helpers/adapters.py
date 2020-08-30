from morse.core.exceptions import MorseServiceError
from morse.core.overlay import MorseOverlay
from morse.builder import Component
from morse.middleware.ros import ROSPublisher, ROSPublisherTF, ROSSubscriber
from morse.middleware.ros_request_manager import ros_service, ros_action
from morse_helpers import morse_local_config as simulation_settings

# Classes already existing in MORSE for publishers/suscribers
#   ROSPublisher 
#   ROSSubscriber
#   ROSPublisherTF
# And our class to create actions and services
#   ROSController
#
# All the components are register using ROSRegister class.


def morse_to_ros_namespace(name):
    return name.replace(".", "/")


class ROSController(MorseOverlay):
    pass


class ROSRegister:

    _middleware_locations = simulation_settings.get_settings().middleware_locations

    @classmethod
    def add_topic(cls, component, name, topic_class=None, **kwargs):

        if topic_class is not None:

            for location in cls._middleware_locations:
                try:
                    class_path = location[1] + topic_class
                    component.add_stream("ros", class_path, topic = name, **kwargs)
                    break
                except:
                    pass

        else:
            component.add_stream("ros", topic = name, **kwargs)
        
    @classmethod
    def add_controller(cls, component, name, controller_class):

        for location in cls._middleware_locations:
            try:
                class_path = location[1] + controller_class
                component.add_overlay("ros", class_path, namespace = name)
                break
            except:
                pass
