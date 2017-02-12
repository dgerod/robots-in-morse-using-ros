# =======================================================================================
# Kuka LWR with ROS
# =======================================================================================

from morse_helpers import morse_local_config as local_settings
local_settings.configure_simulation(__file__)
from morse_helpers.storage import FileStorage
from morse_helpers.adapters import ROSRegister

from morse.builder import Environment
from morse.builder import FakeRobot
from morse.builder.actuators import KukaLWR
from morse.builder.sensors import ArmaturePose

# ---------------------------------------------------------------------------------------
        
def create_simulation():
    
    # Create the robot
    # ----------------------------------------------------------

    robot = FakeRobot() 
    robot.name = "kuka_lwr"

    # Add robot arm to the robot 
    arm = KukaLWR()    
    arm_pose = ArmaturePose()
    arm.append(arm_pose)        
    robot.append(arm)    
    
    # Set-up ROS connection 
    # ----------------------------------------------------------
    
    robot.add_default_interface('ros')

    ROSRegister.add_topic(arm_pose, "klwr/joint_states", "ArmStatePublisher")
    ROSRegister.add_controller(arm, "klwr", "ArmCtrlByActions")

    # Environment 
    # ----------------------------------------------------------
    
    env = Environment(FileStorage.find("empty_world.blend"))
    env.set_camera_location([2.0, -2.0, 4.0])

    env.show_framerate(True)
    
# ---------------------------------------------------------------------------------------

create_simulation()

# =======================================================================================
