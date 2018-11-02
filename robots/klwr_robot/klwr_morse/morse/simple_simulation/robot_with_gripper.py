from morse_helpers import morse_local_config as local_settings
local_settings.load(__file__)

from morse_helpers.settings import SimulationLocalSettings
from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister

from morse.builder import Clock
from morse.builder import Environment, FakeRobot
from morse.builder.actuators import KukaLWR, Gripper
from morse.builder.sensors import ArmaturePose


def create_simulation():

    # local_settings._middleware_locations = local_settings._middleware_locations[1:]
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

    ROSRegister.add_topic(arm_pose, "klwr/joint_states", "ArmStatePublisherForKLWR")
    ROSRegister.add_controller(arm, "klwr", "ArmControllerForKLWR")

    # Environment
    # ----------------------------------------------------------

    fileStorage = FileStorage()
    env = Environment(fileStorage.find("environments/empty_world.blend"))

    env.set_camera_location([2.0, -2.0, 4.0])
    env.show_framerate(True)


create_simulation()
