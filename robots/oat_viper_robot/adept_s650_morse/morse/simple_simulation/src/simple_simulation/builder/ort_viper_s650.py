import math

from morse.builder import FakeRobot, Pose, ArmaturePose
from morse_helpers.adapters import ROSRegister
from simple_simulation.builder.base.actuators import OrtViperS650
from simple_simulation.builder.base.actuators import Gripper


def _create_robot_with_gripper(name):

    base = FakeRobot()
    base.name = name
    base_pose = Pose()
    base.append(base_pose)

    arm = OrtViperS650()
    base.append(arm)
    arm_pose = ArmaturePose()
    arm.append(arm_pose)

    """
    gripper = Gripper()
    gripper.properties(Angle=10.0, Distance=0.05)
    gripper.translate(z=1.15)
    arm.append(gripper)
    """

    from collections import namedtuple
    RoboticSystem = namedtuple("RoboticSystem", "base, base_pose, arm, arm_pose, gripper, camera")
    robot = RoboticSystem(base, base_pose, arm, arm_pose, None, None)

    return robot


def _add_ros_controllers_to_robot(robot):

    robot.base.add_default_interface('ros')

    ROSRegister.add_topic(robot.base_pose, "not_used", "PoseTfPublisher",
                          frame_id='map', child_frame_id="base_link")

    ROSRegister.add_controller(robot.arm, "ort_viper_s650", "OrtViperS650TrajectoryController")
    ROSRegister.add_topic(robot.arm_pose, "ort_viper_s650/joint_states", "OrtViperS650RobotStatePublisher")
    #ROSRegister.add_controller(robot.gripper, "viper", 'GripperController')


def create_robot():

    robot = _create_robot_with_gripper("viper")
    _add_ros_controllers_to_robot(robot)
    return robot

