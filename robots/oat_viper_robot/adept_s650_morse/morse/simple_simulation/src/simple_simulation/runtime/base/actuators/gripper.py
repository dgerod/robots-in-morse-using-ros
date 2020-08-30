import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator
from morse.core import blenderapi
from morse.core.services import service
from morse.helpers.components import add_data, add_property


class Gripper(morse.core.actuator.Actuator):
    """
    Actuator capable of grabbing objects marked with the ``Graspable``
      Game Property.  Currently it only works using services: **grab** and
      **release**.  When instructed to grab an object, it will check if it
      is within range, and if so, will parent the grabbed object to
      itself.

    .. note::

        For objects to be detected and grabbed by the gripper, they must
        have the following settings in the **Physics Properties** panel:

            - **Actor** must be checked
            - **Collision Bounds** must be checked
            - **Physics Type** must be ``Rigid Body``

        This will work even for Static objects.

    .. warning::

        This actuator does not simulate the physical interaction of the
          gripper fingers with the objects it grabs. Its purpose is to
          abstract the action of taking an object, for high level
          interaction experiments.

        And angle (radiants) and distance of the radar cannot be modified in execution.
          These properties are set in the build script based on user input.

    """

    _name = "Gripper"
    _short_desc = ""

    add_data('grab', False, "bool", "Currently not used")

    # These properties are not used directly in the logic, but are used
    # in the builder to create the radar properly.
    # These value cannot be changed dynamically in bge.
    add_property('_angle', 1.0472, 'Angle', 'float',
                 'Aperture angle of the radar capable to detecting the \
                  graspable objects (in degree)')
    add_property('_distance', 0.5, 'Distance', 'float',
                 'Detection distance in meter. Graspable objects further \
                 way from the gripper than this distance cannot be  \
                 held')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        super().__init__(obj, parent)

        self._controller = self.bge_object.controllers[0]
        self._animations = {
            'open': self._controller.actuators['Close_anim'],
            'close': self._controller.actuators['Open_anim']
        }
        self._animating = ''

        self._radar = self._controller.sensors['Radar']

        assert(self._radar.angle, self._angle)
        assert (self._radar.distance, self._distance)

        self._nearest_object = None
        self._grabbed_object = None

        logger.info('Component initialized')
        logger.setLevel(logging.DEBUG)

    def find_object(self):
        """
        Store the object that is within reach of the gripper.
           Uses a Blender Radar Sensor to detect objects with the 'Graspable'
           property in front of this component
        """

        radar = blenderapi.controller().sensors['Radar']
        self._nearest_object = None

        if radar.triggered and radar.positive:
            min_distance = 100
            for test_obj in radar.hitObjectList:
                new_distance = self.bge_object.getDistanceTo(test_obj)
                if new_distance < min_distance:
                    self._nearest_object = test_obj
                    min_distance = new_distance

    @service
    def grab(self):
        """
        Tries to grab an object close to the gripper.

        :returns: if successful (or if an object is already in hand), the
                  name of the object, else None.
        """

        if not self._grabbed_object:

            if self._nearest_object is not None:

                logger.debug("Grabbing object: '%s'" % self._nearest_object)

                # self._near_object.suspendDynamics()

                self._grabbed_object = self._nearest_object
                self._grabbed_object.setParent (self.bge_object)
                self._nearest_object = None

                self._animating = 'close'

                logger.debug("New parent: %s" % self._grabbed_object.parent)
                return self._grabbed_object.name

            else:
                logger.debug("No 'Graspable' object within range of gripper")
                return None
        else:
            logger.debug("Already holding object %s" % self._grabbed_object )
            return self._grabbed_object.name

    @service
    def release(self):
        """
        Free the grabbed object.

        Let it fall down after resetting its rotation.
        Does nothing if no object is held.

        :returns: True if an object has been released, else False (if
                  no object was held).
        """

        if self._grabbed_object is not None:

            self._grabbed_object.removeParent()

            # Place the object on the nearest surface
            #morse.helpers.place_object.do_place(previous_object)
            # Reset rotation of object
            #self._grabbed_object.worldOrientation = [0.0, 0.0, 0.0]
            # Restore Physics simulation
            #previous_object.restoreDynamics()
            #previous_object.setLinearVelocity([0, 0, 0])
            #previous_object.setAngularVelocity([0, 0, 0])

            self._grabbed_object = None
            self._animating = 'open'

            logger.debug("Releasing object: '%s'" % self._nearest_object)

            return True

        else:
            logger.debug("No object currently being held: nothing to release.")
            return False

    def default_action(self):

        self.find_object()

        if self._animating == 'close':

            self._controller.activate(self._animations['close'])
            self._animating = ''
            logger.debug('Playing CLOSE animation')

        elif self._animating == 'open':

            self._controller.activate(self._animations['open'])
            self._animating = ''
            logger.debug('Playing OPEN animation')

        else:
            pass
