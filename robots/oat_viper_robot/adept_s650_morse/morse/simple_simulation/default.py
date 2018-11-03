from morse_helpers import morse_local_config as local_settings
local_settings.load(__file__)

from morse_helpers.settings import SimulationLocalSettings
from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister

from morse.builder import Clock
from morse.builder import Environment
from morse.builder import FakeRobot
from morse.builder.sensors import ArmaturePose

from simple_simulation.builder.actuators import AdeptViperS650


def start_simulation():

    SimulationLocalSettings().show_info()

    # Create the robot
    # ----------------------------------------------------------

    robot = FakeRobot()
    robot.name = "oat_viper_s650"

    arm = AdeptViperS650()
    arm_pose = ArmaturePose()
    arm.append(arm_pose)
    robot.append(arm)

    # Clock to synchronize ROS with MORSE execution
    # ----------------------------------------------------------

    simulation_controller = FakeRobot()
    clock = Clock()
    simulation_controller.append(clock)

    simulation_controller.add_default_interface('ros')
    ROSRegister.add_topic(clock, '/clock')

    # Set-up ROS connection
    # ----------------------------------------------------------

    robot.add_default_interface('ros')
    #robot_namespace = "/" + robot.name + "/"

    ROSRegister.add_topic(arm_pose, 'adept_viper_s650/joint_states', 'StatePublisherForOatViper')
    ROSRegister.add_controller(arm, 'adept_viper_s650', 'ArmControllerForOatViper')

    # Environment
    # ----------------------------------------------------------

    file_storage = FileStorage()
    env = Environment(file_storage.find("empty_world.blend"))
    env.set_camera_location([1.0, -1.0, 2.0])
    env.show_framerate(True)


start_simulation()
