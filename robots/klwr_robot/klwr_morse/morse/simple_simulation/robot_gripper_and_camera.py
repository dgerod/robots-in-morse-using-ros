from morse_helpers import morse_local_config as local_settings
local_settings.load(__file__)

import math

from morse_helpers.settings import SimulationLocalSettings
from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister

from morse.builder import Clock
from morse.builder import Environment, PassiveObject, FakeRobot
from morse.builder.actuators import KukaLWR, Gripper
from morse.builder.sensors import ArmaturePose, SemanticCamera


def start_simulation():

    SimulationLocalSettings().show_info()

    # Create the robot
    # ----------------------------------------------------------

    robot = FakeRobot()
    robot.name = "kuka_lwr"

    arm = KukaLWR()
    arm_pose = ArmaturePose()
    arm.append(arm_pose)

    gripper = Gripper()
    gripper.translate(z=1.15)
    gripper.properties(Angle=90.0, Distance=0.25)
    arm.append(gripper)

    robot.append(arm)

    # Simulation controller
    # ----------------------------------------------------------

    # A simulation controller to which attach all extra sensors used
    # to manage the scene
    simulation_controller = FakeRobot()
    simulation_controller.translate(z=4)

    simulation_controller.add_default_interface('ros')

    # Clock to synchronize ROS with MORSE execution
    clock = Clock()
    simulation_controller.append(clock)

    ROSRegister.add_topic(clock, '/clock')

    # Camera in the top to see all the scene
    top_view = SemanticCamera()
    top_view.rotate(y=-math.pi/2)
    top_view.properties(cam_width=512, cam_height=512, relative=False)
    top_view.properties(tag='box')
    simulation_controller.append(top_view)

    ROSRegister.add_topic(top_view, 'morse/dashboard/top_view')

    # Set-up ROS connection for the robot
    # ----------------------------------------------------------

    robot.add_default_interface('ros')

    ROSRegister.add_topic(arm_pose, "klwr/joint_states", "ArmStatePublisherForKLWR")
    ROSRegister.add_controller(arm, "klwr", "ArmControllerForKLWR")
    ROSRegister.add_controller(gripper, "klwr", 'GripperControllerForKLWR')

    # Environment
    # ----------------------------------------------------------

    file_storage = FileStorage()

    table = PassiveObject(file_storage.find('furnitures.blend'), 'basic_desk')
    table.translate(x=0.25)
    table.rotate(z=-math.pi/2)

    box = PassiveObject(file_storage.find('objects.blend'), 'BlackBox')
    box.name = 'BlackBox'
    box.properties(Type='box', Label=box.name, Graspable=True)
    box.translate(x=0.50, y=0, z=0.75)

    robot.translate(x=0.0, y=-0.0, z=0.75)

    env = Environment(file_storage.find("empty_world.blend"))
    env.set_camera_location([1.0, -1.0, 2.5])
    env.show_framerate(True)


start_simulation()
