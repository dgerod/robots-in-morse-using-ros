from morse_helpers import initialize
initialize(__file__)

import math
from morse_helpers.settings import SimulationLocalSettings
from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister
from morse.builder import Clock
from morse.builder import Environment, PassiveObject, FakeRobot
from simple_simulation.builder.ort_viper_s650 import create_robot


def _create_simulation_controller():

    # A simulation controller to which attach all extra sensors used
    #  to manage the scene
    simulation_controller = FakeRobot("simulation_controller")
    simulation_controller.translate(z=3)

    # Clock to synchronize ROS with MORSE execution
    clock = Clock()
    simulation_controller.append(clock)
    ROSRegister.add_topic(clock, '/clock')


def _prepare_environment(robot):

    file_storage = FileStorage()

    table = PassiveObject(file_storage.find('furnitures.blend'), 'basic_desk')
    table.translate(x=0.25)
    table.rotate(z=-math.pi / 2)

    box = PassiveObject(file_storage.find('objects.blend'), 'BlackBox')
    box.name = 'BlackBox'
    box.properties(Type='box', Label=box.name, Graspable=True)
    box.translate(x=0.50, y=0, z=0.75)

    robot.translate(x=0.0, y=-0.0, z=0.75)

    env = Environment(file_storage.find("empty_world.blend"))
    env.set_camera_location([1.0, -1.0, 2.5])
    env.show_framerate(True)


def start_simulation():

    SimulationLocalSettings().show_info()

    _create_simulation_controller()
    robot = create_robot()
    _prepare_environment(robot.base)


start_simulation()
