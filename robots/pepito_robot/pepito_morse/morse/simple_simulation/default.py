# =======================================================================================
# Pepito with ROS
# =======================================================================================

from morse_helpers import morse_local_config as local_settings
local_settings.configure_simulation(__file__)

from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister

from morse.builder import Environment
from morse.builder import Jido
from morse.builder.actuators import KukaLWR
from morse.builder.sensors import ArmaturePose, Odometry

# ---------------------------------------------------------------------------------------

def create_simulation():

    # Create the robot
    # ----------------------------------------------------------

    robot = Jido()
    robot.name = "pepito"

    odometry = Odometry()
    robot.append(odometry)

    arm = KukaLWR()
    arm.translate(x=0.20, y=-0.15, z=.60)
    arm.rotate(z=3.142)
    arm_pose = ArmaturePose()
    arm.append(arm_pose)
    robot.append(arm)

    # Set-up ROS connection
    # ----------------------------------------------------------

    robot.add_default_interface('ros')

    ROSRegister.add_topic(arm_pose, "pepito/arm/joint_states", "ArmStatePublisher")
    ROSRegister.add_controller(arm, "pepito/arm", "ArmCtrlByActions")
    ROSRegister.add_topic(odometry, "odom", child_frame_id="jido_base_footprint")

    # Environment
    # ----------------------------------------------------------

    env = Environment(FileStorage.find("empty_apartment.blend"))
    env.set_camera_location([2.0, -2.0, 4.0])
    env.show_framerate(True)

# ---------------------------------------------------------------------------------------

create_simulation()

# =======================================================================================

