import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder.creator import ActuatorCreator


class Gripper(ActuatorCreator):

    _classpath = "simple_simulation.base.runtime.actuators.gripper.Gripper"
    _blendname = "gripper"

    GRIPPER_OFFSET = 0.05

    def __init__(self, name=None):

        super().__init__(name,
                         action = ActuatorCreator.USE_BLEND,
                         make_morseable = False)

        self.properties(Angle=1.0472, Distance=0.5)

    def properties(self, **kwargs):

        radar = self._bpy_object.game.sensors["Radar"]

        if 'Angle' in kwargs:
            radar.angle = kwargs['Angle']
        if 'Distance' in kwargs:
            radar.distance = Gripper.GRIPPER_OFFSET + kwargs['Distance']

        ActuatorCreator.properties(self, **kwargs)
